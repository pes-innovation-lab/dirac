#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <memory>
#include <unordered_map>
#include <sstream>
#include "bt/agent_state_db.hpp"
#include "bt/agent_state.hpp"
#include "bt/file_io.hpp"
#include <set>

namespace bt {

class StateUpdater {
public:
    StateUpdater(rclcpp::Node* node, int agent_id, std::shared_ptr<AgentStateDB> db, int total_agents, const std::string& agents_csv_path)
    : node_(node), agent_id_(agent_id), db_(db), total_agents_(total_agents), current_global_tick_(0) {
        // Get all agent IDs from CSV using FileIO
        all_agent_ids_ = bt::FileIO::get_all_agent_ids(agents_csv_path);
        
        RCLCPP_INFO(node_->get_logger(), "StateUpdater loaded %zu agent IDs from CSV", all_agent_ids_.size());
        for (int id : all_agent_ids_) {
            RCLCPP_INFO(node_->get_logger(), "  - Agent ID: %d", id);
        }
        
        // Check if this agent is the leader based on agent state
        is_leader_ = db_->getState(agent_id_).is_leader;
        
        RCLCPP_INFO(node_->get_logger(), "StateUpdater initialized for Agent %d (Leader: %s)", 
                    agent_id_, is_leader_ ? "YES" : "NO");
        
        db_update_pub_ = node_->create_publisher<std_msgs::msg::String>("db_update", 10);
        state_ack_pub_ = node_->create_publisher<std_msgs::msg::String>("state_ack", 10);
        global_tick_pub_ = node_->create_publisher<std_msgs::msg::String>("global_tick", 10);
        
        db_update_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "db_update", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // Parse CSV string to AgentState (all fields)
                std::stringstream ss(msg->data);
                std::string token;
                AgentState state;
                // agent_id
                std::getline(ss, token, ','); state.agent_id = std::stoi(token);
                // current_x
                std::getline(ss, token, ','); state.current_x = std::stoi(token);
                // current_y
                std::getline(ss, token, ','); state.current_y = std::stoi(token);
                // priority
                std::getline(ss, token, ','); state.priority = std::stoi(token);
                // job_id
                std::getline(ss, token, ','); state.job_id = token;
                // goal_x
                std::getline(ss, token, ','); state.goal_x = std::stoi(token);
                // goal_y
                std::getline(ss, token, ','); state.goal_y = std::stoi(token);
                // is_moving
                std::getline(ss, token, ','); state.is_moving = (token == "1" || token == "true");
                // is_leader
                std::getline(ss, token, ','); state.is_leader = (token == "1" || token == "true");
                // current_tick
                std::getline(ss, token, ','); state.current_tick = std::stoi(token);
                // next_moves (semicolon-separated x:y pairs)
                std::getline(ss, token, ',');
                state.next_moves.clear();
                if (!token.empty()) {
                    std::stringstream nmss(token);
                    std::string pairstr;
                    while (std::getline(nmss, pairstr, ';')) {
                        size_t delim = pairstr.find(':');
                        if (delim != std::string::npos) {
                            int x = std::stoi(pairstr.substr(0, delim));
                            int y = std::stoi(pairstr.substr(delim + 1));
                            state.next_moves.emplace_back(x, y);
                        }
                    }
                }
                // goal_reached
                std::getline(ss, token, ','); state.goal_reached = (token == "1" || token == "true");
                // force (x:y)
                std::getline(ss, token, ',');
                {
                    size_t delim = token.find(':');
                    if (delim != std::string::npos) {
                        state.force.first = std::stod(token.substr(0, delim));
                        state.force.second = std::stod(token.substr(delim + 1));
                    } else {
                        state.force = {0.0, 0.0};
                    }
                }
                // chain_force (x:y)
                std::getline(ss, token, ',');
                {
                    size_t delim = token.find(':');
                    if (delim != std::string::npos) {
                        state.chain_force.first = std::stod(token.substr(0, delim));
                        state.chain_force.second = std::stod(token.substr(delim + 1));
                    } else {
                        state.chain_force = {0.0, 0.0};
                    }
                }
                // stuck_counter
                std::getline(ss, token, ','); state.stuck_counter = std::stoi(token);
                // force_multiplier
                std::getline(ss, token, ','); state.force_multiplier = std::stod(token);
                // timestamp (optional, set to now if not present)
                if (std::getline(ss, token, ',')) {
                    try {
                        state.timestamp = std::chrono::system_clock::time_point(std::chrono::milliseconds(std::stoll(token)));
                    } catch (...) {
                        state.timestamp = std::chrono::system_clock::now();
                    }
                } else {
                    state.timestamp = std::chrono::system_clock::now();
                }
                // Track received agent IDs for this tick
                int tick = state.current_tick;
                received_agents_by_tick_[tick].insert(state.agent_id);
                
                // Only update state if it's not our own agent (preserve local agent's leadership)
                if (state.agent_id != agent_id_) {
                    db_->setState(state.agent_id, state);
                } else {
                    // For our own agent, preserve leadership
                    db_->setStatePreserveLeadership(agent_id_, state);
                }
                
                RCLCPP_DEBUG(node_->get_logger(), "Agent %d received state update from agent %d for tick %d", 
                            agent_id_, state.agent_id, tick);
                
                // Check if we should send ack now that we've received this state
                check_and_send_ack_if_ready(tick);
            }
        );

        // Subscribe to state acknowledgments (leader only)
        if (is_leader_) {
            RCLCPP_INFO(node_->get_logger(), "Agent %d is LEADER - subscribing to state acknowledgments", agent_id_);
            state_ack_sub_ = node_->create_subscription<std_msgs::msg::String>(
                "state_ack", 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    // Parse: "agent_id,tick"
                    std::stringstream ss(msg->data);
                    std::string token;
                    std::getline(ss, token, ',');
                    int ack_agent_id = std::stoi(token);
                    std::getline(ss, token, ',');
                    int ack_tick = std::stoi(token);
                    
                    // Only process acks for the current tick - ignore old acks
                    if (ack_tick == current_global_tick_) {
                        acks_by_tick_[ack_tick].insert(ack_agent_id);
                        
                        RCLCPP_DEBUG(node_->get_logger(), "Leader received ack from agent %d for current tick %d (%zu/%zu)", 
                                    ack_agent_id, ack_tick, acks_by_tick_[ack_tick].size(), all_agent_ids_.size());
                        
                        // Only advance when all agents have acknowledged the current tick
                        if (acks_by_tick_[ack_tick].size() == all_agent_ids_.size()) {
                            RCLCPP_INFO(node_->get_logger(), "Leader: All %zu agents acknowledged current tick %d, advancing global tick", 
                                       all_agent_ids_.size(), ack_tick);
                            
                            // Clear old tick data to prevent memory buildup
                            acks_by_tick_.erase(ack_tick);
                            received_agents_by_tick_.erase(ack_tick);
                            
                            advance_global_tick();
                        }
                    } else {
                        RCLCPP_DEBUG(node_->get_logger(), "Leader ignoring ack from agent %d for old tick %d (current: %d)", 
                                    ack_agent_id, ack_tick, current_global_tick_);
                    }
                }
            );
        }

        // Subscribe to global tick updates
        global_tick_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "global_tick", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                int new_tick = std::stoi(msg->data);
                if (new_tick > current_global_tick_) {
                    current_global_tick_ = new_tick;
                    // Notify agent of tick change
                    if (tick_change_callback_) {
                        tick_change_callback_(current_global_tick_);
                    }
                }
            }
        );
    }

    void set_tick_change_callback(std::function<void(int)> callback) {
        tick_change_callback_ = callback;
    }

    void send_state_ack(int tick) {
        std_msgs::msg::String msg;
        msg.data = std::to_string(agent_id_) + "," + std::to_string(tick);
        state_ack_pub_->publish(msg);
        RCLCPP_DEBUG(node_->get_logger(), "Agent %d sent acknowledgment for tick %d", agent_id_, tick);
    }

    int get_current_global_tick() const {
        return current_global_tick_;
    }
    
    void publish_state_for_tick(int tick) {
        // First, immediately publish our own state (write first)
        if (!own_state_written_for_tick_[tick]) {
            maybe_write_own_state_immediately(tick);
        }
        
        // Then check if we should send ack (after all reads are complete)
        check_and_send_ack_if_ready(tick);
    }

private:
    void advance_global_tick() {
        current_global_tick_++;
        std_msgs::msg::String msg;
        msg.data = std::to_string(current_global_tick_);
        global_tick_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Leader agent %d advanced global tick to %d", agent_id_, current_global_tick_);
    }

    void maybe_write_own_state(int tick) {
        // This is the old method - now split into immediate write + conditional ack
        maybe_write_own_state_immediately(tick);
        check_and_send_ack_if_ready(tick);
    }
    
    void maybe_write_own_state_immediately(int tick) {
        if (own_state_written_for_tick_[tick]) {
            return; // Already written for this tick
        }
        
        std_msgs::msg::String msg;
        std::stringstream ss;
        // Write own agent state to CSV format
        ss << agent_id_ << ','
           << db_->getState(agent_id_).current_x << ','
           << db_->getState(agent_id_).current_y << ','
           << db_->getState(agent_id_).priority << ','
           << db_->getState(agent_id_).job_id << ','
           << db_->getState(agent_id_).goal_x << ','
           << db_->getState(agent_id_).goal_y << ','
           << (db_->getState(agent_id_).is_moving ? "true" : "false") << ','
           << (db_->getState(agent_id_).is_leader ? "true" : "false") << ','
           << tick << ',';
        const auto& next_moves = db_->getState(agent_id_).next_moves;
        for (size_t i = 0; i < next_moves.size(); ++i) {
            ss << next_moves[i].first << ':' << next_moves[i].second;
            if (i < next_moves.size() - 1) {
                ss << ';';
            }
        }
        ss << ','
           << (db_->getState(agent_id_).goal_reached ? "true" : "false") << ','
           << db_->getState(agent_id_).force.first << ':' << db_->getState(agent_id_).force.second << ','
           << db_->getState(agent_id_).chain_force.first << ':' << db_->getState(agent_id_).chain_force.second << ','
           << db_->getState(agent_id_).stuck_counter << ','
           << db_->getState(agent_id_).force_multiplier << ','
           << std::chrono::duration_cast<std::chrono::milliseconds>(db_->getState(agent_id_).timestamp.time_since_epoch()).count();
        msg.data = ss.str();
        db_update_pub_->publish(msg);
        own_state_written_for_tick_[tick] = true;
        
        RCLCPP_INFO(node_->get_logger(), "Agent %d published state for tick %d", agent_id_, tick);
        
        // Mark that we've "received" our own state for this tick
        received_agents_by_tick_[tick].insert(agent_id_);
    }
    
    void check_and_send_ack_if_ready(int tick) {
        // Send ack only when we've written our state AND received all other states
        if (own_state_written_for_tick_[tick] && received_agents_by_tick_[tick].size() == all_agent_ids_.size()) {
            send_state_ack(tick);
        }
    }

    rclcpp::Node* node_;
    int agent_id_;
    std::shared_ptr<AgentStateDB> db_;
    int total_agents_;
    bool is_leader_;
    int current_global_tick_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr db_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_ack_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_tick_pub_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr db_update_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_ack_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr global_tick_sub_;
    
    std::set<int> all_agent_ids_;
    std::unordered_map<int, std::set<int>> received_agents_by_tick_;
    std::unordered_map<int, std::set<int>> acks_by_tick_;
    std::unordered_map<int, bool> own_state_written_for_tick_;
    
    std::function<void(int)> tick_change_callback_;
};

} // namespace bt