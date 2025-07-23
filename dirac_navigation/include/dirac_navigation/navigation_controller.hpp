#ifndef DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
#define DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include "dirac_navigation/strategies/movement_strategy.hpp"
#include "dirac_navigation/strategies/movement_strategy_factory.hpp"
#include "dirac_navigation/publishers/publisher_manager.hpp"
#include "dirac_navigation/config/navigation_config.hpp"
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
     * @param config Optional navigation configuration (will load from parameters if not provided)
     */
    explicit NavigationController(rclcpp::Node::SharedPtr node, 
                                 const config::NavigationConfig& config = config::NavigationConfig{});

    /**
     * @brief Initialize from ROS parameters
     * @param node ROS2 node to load parameters from
     * @return True if successfully initialized
     */
    bool initializeFromParameters(rclcpp::Node::SharedPtr node);

    /**
     * @brief Update configuration
     * @param config New navigation configuration
     */
    void updateConfiguration(const config::NavigationConfig& config);

    /**
     * @brief Get current configuration
     * @return Current navigation configuration
     */
    const config::NavigationConfig& getConfiguration() const;

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
    
    // Configuration management
    config::NavigationConfig config_;
    
    // Publisher manager for handling multiple agent publishers
    std::unique_ptr<publishers::PublisherManager> publisher_manager_;

    // Private helper methods    
    /**
     * @brief Create movement context for strategy execution
     * @param agent_id ID of the agent
     * @return Movement context with all necessary information
     */
    strategies::MovementContext createMovementContext(int32_t agent_id);
    
    /**
     * @brief Apply configuration to internal components
     */
    void applyConfiguration();
};

} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_NAVIGATION_CONTROLLER_HPP_
