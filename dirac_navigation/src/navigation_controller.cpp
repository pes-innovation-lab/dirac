#include "dirac_navigation/navigation_controller.hpp"

namespace dirac_navigation
{

NavigationController::NavigationController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    if (!node_) {
        throw std::invalid_argument("Node cannot be null");
    }
    
    // Default movement parameters - now using strategy pattern structure
    movement_parameters_.linear_speed = 2.0;      // Linear velocity
    movement_parameters_.angular_speed = 1.57;    // Angular velocity (π/2 rad/s for 90° turns)
    movement_parameters_.move_distance = 1.0;     // Distance to move forward/backward (1 unit)
    movement_parameters_.turn_angle = 1.57;       // 90 degrees in radians (π/2)
    
    // Initialize publisher manager with default configuration
    publishers::TopicConfig default_config;
    default_config.base_topic = "cmd_vel";
    default_config.simulation_mode = true;
    
    publisher_manager_ = std::make_unique<publishers::PublisherManager>(node_, default_config);

    RCLCPP_INFO(node_->get_logger(), "Navigation Controller library initialized with strategy pattern and publisher manager");
}

void NavigationController::set_movement_parameters(double linear_speed, double angular_speed, double move_distance)
{
    movement_parameters_.linear_speed = linear_speed;
    movement_parameters_.angular_speed = angular_speed;
    movement_parameters_.move_distance = move_distance;
    // Keep turn_angle consistent with angular_speed (90 degrees)
    movement_parameters_.turn_angle = 1.57;
    
    RCLCPP_INFO(node_->get_logger(), "Movement parameters updated: linear=%.2f, angular=%.2f, distance=%.2f", 
                movement_parameters_.linear_speed, movement_parameters_.angular_speed, movement_parameters_.move_distance);
}

void NavigationController::set_cmd_vel_topic(const std::string& topic_name)
{
    publishers::TopicConfig config;
    config.base_topic = topic_name;
    config.simulation_mode = true; // Default to simulation mode when topic changes
    
    publisher_manager_->updateConfig(config);
    RCLCPP_INFO(node_->get_logger(), "Command velocity topic updated to: %s", topic_name.c_str());
}

void NavigationController::set_simulation_mode(bool is_simulation)
{
    publishers::TopicConfig config;
    config.simulation_mode = is_simulation;
    config.base_topic = "cmd_vel"; // Keep current base topic
    
    publisher_manager_->updateConfig(config);
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
    context.parameters = movement_parameters_;
    context.agent_id = agent_id;
    context.completion_callback = nullptr; // Can be set by caller if needed
    
    return context;
}

} // namespace dirac_navigation
