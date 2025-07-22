#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <memory>
#include <unordered_map>
#include <sstream>
#include "bt/agent_state_db.hpp"
#include "bt/agent_state.hpp"

namespace bt {

class StateUpdater {
public:
    StateUpdater(rclcpp::Node* node, int agent_id, std::shared_ptr<AgentStateDB> db, int total_agents)
    : node_(node), agent_id_(agent_id), db_(db), total_agents_(total_agents) {
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
                // Only allow one update per agent per tick
                int agent_id = state.agent_id;
                int tick = state.current_tick;
                auto& last_tick = last_tick_by_agent_[agent_id];
                if (last_tick != tick) {
                    db_->setState(agent_id, state);
                    last_tick = tick;
                    // --- New logic for peer update collection ---
                    if (tick != current_tick_) {
                        // New tick: reset peer set and start timer
                        current_tick_ = tick;
                        peer_updates_.clear();
                        if (timer_) timer_->cancel();
                        timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this, tick]() {
                            this->maybe_write_own_state(tick);
                        });
                    }
                    if (agent_id != agent_id_) {
                        peer_updates_.insert(agent_id);
                    }
                    // Check if 95% of (n-1) peers have updated
                    int peer_count = total_agents_ - 1;
                    int threshold = static_cast<int>(0.95 * peer_count);
                    if (static_cast<int>(peer_updates_.size()) >= threshold && !own_state_written_for_tick_[tick]) {
                        if (timer_) timer_->cancel();
                        maybe_write_own_state(tick);
                    }
                    RCLCPP_INFO(node_->get_logger(), "DB updated for agent %d at tick %d", agent_id, tick);
                } else {
                    RCLCPP_DEBUG(node_->get_logger(), "Duplicate update for agent %d at tick %d ignored", agent_id, tick);
                }
            }
        );
    }

private:
    void maybe_write_own_state(int tick) {
        if (!own_state_written_for_tick_[tick]) {
            own_state_written_for_tick_[tick] = true;
            // Serialize this agent's state for this tick
            AgentState state = db_->getState(agent_id_);
            std_msgs::msg::String msg;
            msg.data = agent_state_to_csv(state);
            db_update_pub_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "Agent %d writing own state for tick %d (peer updates: %zu)", agent_id_, tick, peer_updates_.size());
        }
    }
    std::string agent_state_to_csv(const AgentState& state) {
        std::ostringstream oss;
        oss << state.agent_id << ','
            << state.current_x << ','
            << state.current_y << ','
            << state.priority << ','
            << state.job_id << ','
            << state.goal_x << ','
            << state.goal_y << ','
            << (state.is_moving ? "1" : "0") << ','
            << (state.is_leader ? "1" : "0") << ','
            << state.current_tick << ',';
        // next_moves as semicolon-separated x:y pairs
        for (size_t i = 0; i < state.next_moves.size(); ++i) {
            oss << state.next_moves[i].first << ':' << state.next_moves[i].second;
            if (i + 1 < state.next_moves.size()) oss << ';';
        }
        oss << ',' << (state.goal_reached ? "1" : "0") << ',';
        oss << state.force.first << ':' << state.force.second << ',';
        oss << state.chain_force.first << ':' << state.chain_force.second << ',';
        oss << state.stuck_counter << ',';
        oss << state.force_multiplier << ',';
        // timestamp as ms since epoch
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(state.timestamp.time_since_epoch()).count();
        oss << ms;
        return oss.str();
    }
    rclcpp::Node* node_;
    int agent_id_;
    std::shared_ptr<AgentStateDB> db_;
    int total_agents_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr db_update_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr db_update_pub_;
    std::unordered_map<int, int> last_tick_by_agent_;
    int current_tick_ = -1;
    std::set<int> peer_updates_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int, bool> own_state_written_for_tick_;
};

} // namespace bt
