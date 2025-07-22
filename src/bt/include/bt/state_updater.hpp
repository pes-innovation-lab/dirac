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
    StateUpdater(rclcpp::Node* node, int agent_id, std::shared_ptr<AgentStateDB> db, int total_agents)
    : node_(node), agent_id_(agent_id), db_(db), total_agents_(total_agents) {
        // Get all agent IDs from CSV using FileIO
        all_agent_ids_ = bt::FileIO::get_all_agent_ids("../agents.csv");
        db_update_pub_ = node_->create_publisher<std_msgs::msg::String>("db_update", 10);
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
                db_->setState(state.agent_id, state);
                // If all agent IDs have been received for this tick, publish own state
                if (received_agents_by_tick_[tick].size() == all_agent_ids_.size() && !own_state_written_for_tick_[tick]) {
                    maybe_write_own_state(tick);
                }
            }
        );
    }

private:
    rclcpp::Node* node_;
    int agent_id_;
    std::shared_ptr<AgentStateDB> db_;
    int total_agents_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr db_update_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr db_update_sub_;
    std::set<int> all_agent_ids_;
    std::unordered_map<int, std::set<int>> received_agents_by_tick_;
    std::unordered_map<int, bool> own_state_written_for_tick_;

    void maybe_write_own_state(int tick) {
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
    }
};

}