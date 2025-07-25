#ifndef DIRAC_NAVIGATION_CONFIG_NAVIGATION_CONFIG_HPP_
#define DIRAC_NAVIGATION_CONFIG_NAVIGATION_CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>
#include "dirac_navigation/strategies/movement_strategy.hpp"
#include "dirac_navigation/publishers/publisher_manager.hpp"
#include <string>

namespace dirac_navigation
{
namespace config
{

/**
 * @brief Configuration for controller behavior
 */
struct ControllerConfig
{
    bool enable_controller = true;
    
    /**
     * @brief Load controller configuration from ROS parameters
     * @param node ROS2 node to get parameters from
     * @param prefix Parameter prefix (default: "navigation")
     */
    void loadFromParameters(rclcpp::Node::SharedPtr node, const std::string& prefix = "navigation");
    
    /**
     * @brief Validate controller configuration
     * @return True if configuration is valid
     */
    bool isValid() const;
};

/**
 * @brief Complete navigation configuration
 */
class NavigationConfig
{
public:
    /**
     * @brief Default constructor with sensible defaults
     */
    NavigationConfig();
    
    /**
     * @brief Load configuration from ROS parameters
     * @param node ROS2 node to get parameters from
     * @return True if successfully loaded
     */
    bool loadFromParameters(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Validate the entire configuration
     * @return True if all configurations are valid
     */
    bool isValid() const;
    
    /**
     * @brief Log the current configuration
     * @param node ROS2 node for logging
     */
    void logConfiguration(rclcpp::Node::SharedPtr node) const;
    
    // Configuration components
    ControllerConfig controller;
    strategies::MovementParameters movement;
    publishers::TopicConfig topics;

private:
    /**
     * @brief Declare all parameters with default values
     * @param node ROS2 node to declare parameters on
     */
    void declareParameters(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Load movement parameters from ROS parameters
     * @param node ROS2 node to get parameters from
     */
    void loadMovementParameters(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Load topic parameters from ROS parameters
     * @param node ROS2 node to get parameters from
     */
    void loadTopicParameters(rclcpp::Node::SharedPtr node);
};

/**
 * @brief Factory for creating navigation configurations
 */
class NavigationConfigFactory
{
public:
    /**
     * @brief Create configuration from ROS parameters
     * @param node ROS2 node to get parameters from
     * @return Navigation configuration
     * @throws std::runtime_error if configuration is invalid
     */
    static NavigationConfig createFromParameters(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Create default configuration
     * @return Default navigation configuration
     */
    static NavigationConfig createDefault();
    
    /**
     * @brief Validate and create configuration from parameters
     * @param node ROS2 node to get parameters from
     * @return Pair of (config, is_valid)
     */
    static std::pair<NavigationConfig, bool> tryCreateFromParameters(rclcpp::Node::SharedPtr node);
};

} // namespace config
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_CONFIG_NAVIGATION_CONFIG_HPP_
