#include "dirac_navigation/navigation_controller.hpp"

namespace dirac_navigation
{

NavigationController::NavigationController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Default movement parameters
    linear_speed_ = 2.0;   // Linear velocity
    angular_speed_ = 1.57; // Angular velocity (π/2 rad/s for 90° turns)
    move_distance_ = 1.0;  // Distance to move forward/backward (1 unit)
    turn_angle_ = 1.57;    // 90 degrees in radians (π/2)
    cmd_vel_topic_ = "cmd_vel"; // Default topic name
    is_simulation_mode_ = true; // Default to simulation mode

    RCLCPP_INFO(node_->get_logger(), "Navigation Controller library initialized");
}

void NavigationController::set_movement_parameters(double linear_speed, double angular_speed, double move_distance)
{
    linear_speed_ = linear_speed;
    angular_speed_ = angular_speed;
    move_distance_ = move_distance;
    
    RCLCPP_INFO(node_->get_logger(), "Movement parameters updated: linear=%.2f, angular=%.2f, distance=%.2f", 
                linear_speed_, angular_speed_, move_distance_);
}

void NavigationController::set_cmd_vel_topic(const std::string& topic_name)
{
    cmd_vel_topic_ = topic_name;
    RCLCPP_INFO(node_->get_logger(), "Command velocity topic set to: %s", cmd_vel_topic_.c_str());
}

void NavigationController::set_simulation_mode(bool is_simulation)
{
    is_simulation_mode_ = is_simulation;
    RCLCPP_INFO(node_->get_logger(), "Simulation mode set to: %s", is_simulation ? "true (turtle topics)" : "false (namespaced robot topics)");
}

void NavigationController::execute_command(int32_t agent_id, int32_t direction)
{
    RCLCPP_INFO(node_->get_logger(), "Executing command for agent %d: direction %d", agent_id, direction);
    
    switch(direction) {
        case 1: // Forward
            move_forward(agent_id);
            break;
        case 2: // Backward
            move_backward(agent_id);
            break;
        case 3: // Left
            move_left(agent_id);
            break;
        case 4: // Right
            move_right(agent_id);
            break;
        default:
            RCLCPP_WARN(node_->get_logger(), "Invalid direction %d for agent %d. Valid directions: 1=forward, 2=backward, 3=left, 4=right", 
                       direction, agent_id);
    }
}

void NavigationController::move_forward(int32_t agent_id)
{
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Moving forward %.2f units", agent_id, move_distance_);
    
    auto publisher = get_or_create_publisher(agent_id);
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_speed_;
    twist_msg.angular.z = 0.0;
    
    publisher->publish(twist_msg);
    
    // Calculate duration to move exactly the configured distance
    auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
    
    auto timer = node_->create_wall_timer(duration, [this, publisher, agent_id]() {
        stop_turtle(publisher, agent_id);
    });
    
    timers_[agent_id] = timer;
}

void NavigationController::move_backward(int32_t agent_id)
{
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Moving backward %.2f units", agent_id, move_distance_);
    
    auto publisher = get_or_create_publisher(agent_id);
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = -linear_speed_;
    twist_msg.angular.z = 0.0;
    
    publisher->publish(twist_msg);
    
    // Calculate duration to move exactly the configured distance
    auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
    
    auto timer = node_->create_wall_timer(duration, [this, publisher, agent_id]() {
        stop_turtle(publisher, agent_id);
    });
    
    timers_[agent_id] = timer;
}

void NavigationController::move_left(int32_t agent_id)
{
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Moving left (turn-move-turn sequence)", agent_id);
    
    auto publisher = get_or_create_publisher(agent_id);
    
    // Step 1: Turn left 90 degrees
    turn_turtle(publisher, angular_speed_, [this, publisher, agent_id]() {
        // Step 2: Move forward configured distance
        move_forward_then_turn_right(publisher, agent_id);
    });
}

void NavigationController::move_right(int32_t agent_id)
{
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Moving right (turn-move-turn sequence)", agent_id);
    
    auto publisher = get_or_create_publisher(agent_id);
    
    // Step 1: Turn right 90 degrees
    turn_turtle(publisher, -angular_speed_, [this, publisher, agent_id]() {
        // Step 2: Move forward configured distance
        move_forward_then_turn_left(publisher, agent_id);
    });
}

void NavigationController::move_forward_then_turn_right(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_speed_;
    twist_msg.angular.z = 0.0;
    
    publisher->publish(twist_msg);
    
    auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
    
    auto timer = node_->create_wall_timer(duration, [this, publisher, agent_id]() {
        // Step 3: Turn right 90 degrees to reset orientation
        turn_turtle(publisher, -angular_speed_, [this, publisher, agent_id]() {
            stop_turtle(publisher, agent_id);
        });
    });
    
    timers_[agent_id] = timer;
}

void NavigationController::move_forward_then_turn_left(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_speed_;
    twist_msg.angular.z = 0.0;
    
    publisher->publish(twist_msg);
    
    auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
    
    auto timer = node_->create_wall_timer(duration, [this, publisher, agent_id]() {
        // Step 3: Turn left 90 degrees to reset orientation
        turn_turtle(publisher, angular_speed_, [this, publisher, agent_id]() {
            stop_turtle(publisher, agent_id);
        });
    });
    
    timers_[agent_id] = timer;
}

void NavigationController::turn_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, 
                                       double angular_vel, std::function<void()> callback)
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = angular_vel;
    
    publisher->publish(twist_msg);
    
    // Calculate duration to turn exactly 90 degrees
    auto duration = std::chrono::milliseconds(static_cast<int>((turn_angle_ / std::abs(angular_vel)) * 1000));
    
    auto timer = node_->create_wall_timer(duration, callback);
    // Note: We don't store this timer in the map since it's temporary for the sequence
}

void NavigationController::stop_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
{
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    publisher->publish(stop_msg);
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Movement complete", agent_id);
}

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 
NavigationController::get_or_create_publisher(int32_t agent_id)
{
    // Check if publisher already exists for this agent
    auto it = publishers_.find(agent_id);
    if (it != publishers_.end()) {
        return it->second;
    }
    
    // Create new publisher for this agent using configured topic name
    std::string topic_name;
    
    if (is_simulation_mode_) {
        // Simulation mode: publish to turtle-specific topics
        topic_name = "/turtle" + std::to_string(agent_id) + "/" + cmd_vel_topic_;
    } else {
        // Non-simulation mode: publish to namespaced robot topics
        topic_name = "/robot" + std::to_string(agent_id) + "/" + cmd_vel_topic_;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Creating publisher for agent %d on topic '%s'", agent_id, topic_name.c_str());
    
    auto publisher = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
    
    publishers_[agent_id] = publisher;
    
    RCLCPP_INFO(node_->get_logger(), "Created publisher for agent %d on topic '%s'", 
                agent_id, topic_name.c_str());
    
    return publisher;
}

} // namespace dirac_navigation
