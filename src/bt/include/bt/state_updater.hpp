#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include "bt/agent_state.hpp"

namespace bt {

class StateUpdater {
public:
    StateUpdater(rclcpp::Node* node, int agent_id, const AgentState& agent_state)
    : node_(node), agent_id_(agent_id), is_leader_(agent_id == 1), current_tick_(0), last_tick_(-1), agent_state_(agent_state) {
        db_update_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "db_update", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // Deserialize and process agent state here
                RCLCPP_INFO(node_->get_logger(), "Received db_update: %s", msg->data.c_str());
            }
        );
        db_update_pub_ = node_->create_publisher<std_msgs::msg::String>("db_update", 10);

        if (!is_leader_) {
            global_tick_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
                "global_tick", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    current_tick_ = msg->data;
                    RCLCPP_INFO(node_->get_logger(), "Received global_tick: %d", current_tick_);
                }
            );
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(200),
                [this]() { this->check_tick(); }
            );
        }
    }

    bool is_leader() const { return is_leader_; }
    int get_current_tick() const { return current_tick_; }

private:
    void check_tick() {
        if (current_tick_ != last_tick_) {
            last_tick_ = current_tick_;
            // Serialize agent_state_ to string (simple CSV for now)
            std::stringstream ss;
            ss << agent_state_.agent_id << "," << agent_state_.current_x << "," << agent_state_.current_y << "," << agent_state_.priority << "," << agent_state_.job_id << "," << agent_state_.goal_x << "," << agent_state_.goal_y << "," << agent_state_.current_tick;
            auto msg = std_msgs::msg::String();
            msg.data = ss.str();
            db_update_pub_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "Published db_update: %s", msg.data.c_str());
        }
    }

    rclcpp::Node* node_;
    int agent_id_;
    bool is_leader_;
    int current_tick_;
    int last_tick_;
    AgentState agent_state_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr global_tick_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr db_update_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr db_update_sub_;
};

} // namespace bt
