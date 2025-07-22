#ifndef DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
#define DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include <map>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>

namespace dirac_navigation
{

class NavigationController
{
public:
    /**
     * @brief Constructor for NavigationController
     * @param node Shared pointer to ROS2 node for creating publishers and timers
     */
    explicit NavigationController(rclcpp::Node::SharedPtr node);

    /**
     * @brief Move turtle forward by configured distance
     * @param agent_id ID of the agent/turtle to move
     */
    void move_forward(int32_t agent_id);

    /**
     * @brief Move turtle backward by configured distance
     * @param agent_id ID of the agent/turtle to move
     */
    void move_backward(int32_t agent_id);

    /**
     * @brief Move turtle left (turn left 90°, move forward, turn right 90°)
     * @param agent_id ID of the agent/turtle to move
     */
    void move_left(int32_t agent_id);

    /**
     * @brief Move turtle right (turn right 90°, move forward, turn left 90°)
     * @param agent_id ID of the agent/turtle to move
     */
    void move_right(int32_t agent_id);

    /**
     * @brief Execute movement command based on direction
     * @param agent_id ID of the agent/turtle to move
     * @param direction Direction code (1=forward, 2=backward, 3=left, 4=right)
     */
    void execute_command(int32_t agent_id, int32_t direction);

    /**
     * @brief Set movement parameters
     * @param linear_speed Linear velocity for movements
     * @param angular_speed Angular velocity for turns
     * @param move_distance Distance to move in units
     */
    void set_movement_parameters(double linear_speed, double angular_speed, double move_distance);

    /**
     * @brief Set the command velocity topic name
     * @param topic_name Base topic name for cmd_vel
     */
    void set_cmd_vel_topic(const std::string& topic_name);

    /**
     * @brief Set simulation mode to determine topic naming strategy
     * @param is_simulation True for simulation mode (turtle topics), false for real robot mode (namespaced topics)
     */
    void set_simulation_mode(bool is_simulation);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Map of agent_id to their respective cmd_vel publishers
    std::map<int32_t, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    
    // Map of agent_id to their timers (to keep timers alive)
    std::map<int32_t, rclcpp::TimerBase::SharedPtr> timers_;
    
    // Movement parameters
    double linear_speed_;
    double angular_speed_;
    double move_distance_;
    double turn_angle_;
    
    // Topic configuration
    std::string cmd_vel_topic_;
    bool is_simulation_mode_;

    // Private helper methods
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr get_or_create_publisher(int32_t agent_id);
    void move_forward_then_turn_right(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id);
    void move_forward_then_turn_left(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id);
    void turn_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, 
                     double angular_vel, std::function<void()> callback);
    void stop_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id);
};

} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
