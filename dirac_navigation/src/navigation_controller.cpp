#include "dirac_navigation/navigation_controller.hpp"
#include <cmath>

namespace dirac_navigation
{

NavigationController::NavigationController(rclcpp::Node::SharedPtr node, 
                                         const config::NavigationConfig& config)
    : node_(node), config_(config)
{
    if (!node_) {
        throw std::invalid_argument("Node cannot be null");
    }
    
    // Validate configuration
    if (!config_.isValid()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid configuration provided, using defaults");
        config_ = config::NavigationConfig{}; // Reset to defaults
    }
    
    // Initialize publisher manager with configuration
    publisher_manager_ = std::make_unique<publishers::PublisherManager>(node_, config_.topics);
    
    // Log the configuration
    config_.logConfiguration(node_);

    RCLCPP_INFO(node_->get_logger(), "Navigation Controller initialized with configuration management");
}

bool NavigationController::initializeFromParameters(rclcpp::Node::SharedPtr node)
{
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("NavigationController"), "Node is null");
        return false;
    }
    
    try {
        auto new_config = config::NavigationConfigFactory::createFromParameters(node);
        updateConfiguration(new_config);
        RCLCPP_INFO(node_->get_logger(), "Successfully initialized from ROS parameters");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize from parameters: %s", e.what());
        return false;
    }
}

void NavigationController::updateConfiguration(const config::NavigationConfig& config)
{
    if (!config.isValid()) {
        RCLCPP_ERROR(node_->get_logger(), "Attempted to update with invalid configuration");
        return;
    }
    
    config_ = config;
    applyConfiguration();
    
    RCLCPP_INFO(node_->get_logger(), "Configuration updated successfully");
    config_.logConfiguration(node_);
}

const config::NavigationConfig& NavigationController::getConfiguration() const
{
    return config_;
}

void NavigationController::applyConfiguration()
{
    // Update publisher manager with new topic configuration
    if (publisher_manager_) {
        publisher_manager_->updateConfig(config_.topics);
    }
}

void NavigationController::set_movement_parameters(double linear_speed, double angular_speed, double move_distance)
{
    config_.movement.linear_speed = linear_speed;
    config_.movement.angular_speed = angular_speed;
    config_.movement.move_distance = move_distance;
    // Keep turn_angle consistent (90 degrees)
    config_.movement.turn_angle = M_PI / 2.0;
    
    RCLCPP_INFO(node_->get_logger(), "Movement parameters updated: linear=%.2f, angular=%.2f, distance=%.2f", 
                config_.movement.linear_speed, config_.movement.angular_speed, config_.movement.move_distance);
}

void NavigationController::set_cmd_vel_topic(const std::string& topic_name)
{
    config_.topics.base_topic = topic_name;
    applyConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Command velocity topic updated to: %s", topic_name.c_str());
}

void NavigationController::set_simulation_mode(bool is_simulation)
{
    config_.topics.simulation_mode = is_simulation;
    applyConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Simulation mode updated to: %s", 
                is_simulation ? "true (turtle topics)" : "false (namespaced robot topics)");
}

void NavigationController::execute_command(int32_t agent_id, int32_t direction)
{
    RCLCPP_INFO(node_->get_logger(), "Executing command for agent %d: direction %d", agent_id, direction);
    
    // Create strategy using factory
    auto strategy = strategies::MovementStrategyFactory::createStrategy(direction);
    
    if (!strategy) {
        RCLCPP_WARN(node_->get_logger(), 
                   "Invalid direction %d for agent %d. Valid directions: 1=forward, 2=backward, 3=left, 4=right", 
                   direction, agent_id);
        return;
    }
    
    // Create movement context
    strategies::MovementContext context = createMovementContext(agent_id);
    
    // Execute the strategy
    RCLCPP_INFO(node_->get_logger(), "Agent %d: Executing %s strategy", 
               agent_id, strategy->getName().c_str());
    strategy->execute(context);
}

strategies::MovementContext NavigationController::createMovementContext(int32_t agent_id)
{
    strategies::MovementContext context;
    context.publisher = publisher_manager_->getPublisher(agent_id);
    context.node = node_;
    context.parameters = config_.movement;
    context.agent_id = agent_id;
    context.completion_callback = nullptr; // Can be set by caller if needed
    
    return context;
}

} // namespace dirac_navigation
