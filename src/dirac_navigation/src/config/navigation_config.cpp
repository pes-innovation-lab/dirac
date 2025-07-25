#include "dirac_navigation/config/navigation_config.hpp"
#include <stdexcept>
#include <limits>
#include <cmath>

namespace dirac_navigation
{
namespace config
{

void ControllerConfig::loadFromParameters(rclcpp::Node::SharedPtr node, const std::string& prefix)
{
    std::string param_name = prefix + ".enable_controller";
    if (node->has_parameter(param_name)) {
        enable_controller = node->get_parameter(param_name).as_bool();
    }
}

bool ControllerConfig::isValid() const
{
    // Controller config is always valid for now
    return true;
}

NavigationConfig::NavigationConfig()
{
    // Set sensible defaults
    controller.enable_controller = true;
    
    movement.linear_speed = 2.0;
    movement.angular_speed = 1.57;  // π/2 rad/s for 90° turns
    movement.move_distance = 1.0;
    movement.turn_angle = 1.57;     // π/2 radians (90°)
    
    topics.base_topic = "cmd_vel";
    topics.simulation_mode = true;
}

bool NavigationConfig::loadFromParameters(rclcpp::Node::SharedPtr node)
{
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("NavigationConfig"), "Node is null, cannot load parameters");
        return false;
    }
    
    try {
        // Declare parameters first
        declareParameters(node);
        
        // Load controller parameters
        controller.loadFromParameters(node, "navigation");
        
        // Load movement parameters
        loadMovementParameters(node);
        
        // Load topic parameters
        loadTopicParameters(node);
        
        RCLCPP_INFO(node->get_logger(), "Successfully loaded navigation configuration from parameters");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load navigation configuration: %s", e.what());
        return false;
    }
}

bool NavigationConfig::isValid() const
{
    // Validate controller config
    if (!controller.isValid()) {
        return false;
    }
    
    // Validate movement parameters
    if (movement.linear_speed <= 0.0 || movement.linear_speed > 10.0) {
        return false;
    }
    
    if (movement.angular_speed <= 0.0 || movement.angular_speed > 10.0) {
        return false;
    }
    
    if (movement.move_distance <= 0.0 || movement.move_distance > 10.0) {
        return false;
    }
    
    if (movement.turn_angle <= 0.0 || movement.turn_angle > 2 * M_PI) {
        return false;
    }
    
    // Validate topic configuration
    if (topics.base_topic.empty()) {
        return false;
    }
    
    return true;
}

void NavigationConfig::logConfiguration(rclcpp::Node::SharedPtr node) const
{
    if (!node) {
        return;
    }
    
    RCLCPP_INFO(node->get_logger(), "=== Navigation Configuration ===");
    RCLCPP_INFO(node->get_logger(), "Controller:");
    RCLCPP_INFO(node->get_logger(), "  enable_controller: %s", controller.enable_controller ? "true" : "false");
    
    RCLCPP_INFO(node->get_logger(), "Movement:");
    RCLCPP_INFO(node->get_logger(), "  linear_speed: %.3f m/s", movement.linear_speed);
    RCLCPP_INFO(node->get_logger(), "  angular_speed: %.3f rad/s", movement.angular_speed);
    RCLCPP_INFO(node->get_logger(), "  move_distance: %.3f units", movement.move_distance);
    RCLCPP_INFO(node->get_logger(), "  turn_angle: %.3f rad (%.1f°)", movement.turn_angle, movement.turn_angle * 180.0 / M_PI);
    
    RCLCPP_INFO(node->get_logger(), "Topics:");
    RCLCPP_INFO(node->get_logger(), "  base_topic: %s", topics.base_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "  simulation_mode: %s", topics.simulation_mode ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "================================");
}

void NavigationConfig::declareParameters(rclcpp::Node::SharedPtr node)
{
    // Controller parameters
    node->declare_parameter("navigation.enable_controller", controller.enable_controller);
    
    // Movement parameters
    node->declare_parameter("navigation.linear_speed", movement.linear_speed);
    node->declare_parameter("navigation.angular_speed", movement.angular_speed);
    node->declare_parameter("navigation.move_distance", movement.move_distance);
    node->declare_parameter("navigation.turn_angle", movement.turn_angle);
    
    // Topic parameters
    node->declare_parameter("navigation.cmd_vel_topic", topics.base_topic);
    node->declare_parameter("simulation_mode", topics.simulation_mode);
}

void NavigationConfig::loadMovementParameters(rclcpp::Node::SharedPtr node)
{
    movement.linear_speed = node->get_parameter("navigation.linear_speed").as_double();
    movement.angular_speed = node->get_parameter("navigation.angular_speed").as_double();
    movement.move_distance = node->get_parameter("navigation.move_distance").as_double();
    
    // Turn angle might not be in config file, so check if it exists
    if (node->has_parameter("navigation.turn_angle")) {
        movement.turn_angle = node->get_parameter("navigation.turn_angle").as_double();
    } else {
        // Default to 90 degrees if not specified
        movement.turn_angle = M_PI / 2.0;
    }
}

void NavigationConfig::loadTopicParameters(rclcpp::Node::SharedPtr node)
{
    topics.base_topic = node->get_parameter("navigation.cmd_vel_topic").as_string();
    topics.simulation_mode = node->get_parameter("simulation_mode").as_bool();
}

NavigationConfig NavigationConfigFactory::createFromParameters(rclcpp::Node::SharedPtr node)
{
    NavigationConfig config;
    
    if (!config.loadFromParameters(node)) {
        throw std::runtime_error("Failed to load navigation configuration from parameters");
    }
    
    if (!config.isValid()) {
        throw std::runtime_error("Invalid navigation configuration loaded from parameters");
    }
    
    return config;
}

NavigationConfig NavigationConfigFactory::createDefault()
{
    NavigationConfig config;
    // Constructor already sets defaults
    return config;
}

std::pair<NavigationConfig, bool> NavigationConfigFactory::tryCreateFromParameters(rclcpp::Node::SharedPtr node)
{
    NavigationConfig config;
    
    bool loaded = config.loadFromParameters(node);
    bool valid = loaded && config.isValid();
    
    return std::make_pair(config, valid);
}

} // namespace config
} // namespace dirac_navigation
