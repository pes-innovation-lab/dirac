#ifndef DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
#define DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include "dirac_navigation/strategies/movement_strategy.hpp"
#include "dirac_navigation/strategies/movement_strategy_factory.hpp"
#include <map>
#include <string>
#include <memory>

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
     * @brief Execute movement command based on direction using strategy pattern
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
    
    // Movement parameters (now using strategy pattern structure)
    strategies::MovementParameters movement_parameters_;
    
    // Topic configuration
    std::string cmd_vel_topic_;
    bool is_simulation_mode_;

    // Private helper methods
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr get_or_create_publisher(int32_t agent_id);
    
    /**
     * @brief Create movement context for strategy execution
     * @param agent_id ID of the agent
     * @return Movement context with all necessary information
     */
    strategies::MovementContext createMovementContext(int32_t agent_id);
};

} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
