#include "dirac_navigation/strategies/movement_strategy.hpp"

namespace dirac_navigation
{
namespace strategies
{

void MovementStrategy::publishTwist(const MovementContext& context, 
                                   const geometry_msgs::msg::Twist& twist)
{
    if (context.publisher) {
        context.publisher->publish(twist);
        RCLCPP_DEBUG(context.node->get_logger(), 
                    "Agent %d (%s): Published twist - linear.x=%.2f, angular.z=%.2f",
                    context.agent_id, getName().c_str(), twist.linear.x, twist.angular.z);
    } else {
        RCLCPP_ERROR(context.node->get_logger(), 
                    "Agent %d (%s): Publisher is null, cannot publish twist",
                    context.agent_id, getName().c_str());
    }
}

void MovementStrategy::createMovementTimer(const MovementContext& context, 
                                         std::chrono::milliseconds duration,
                                         std::function<void()> callback)
{
    if (context.node) {
        context.node->create_wall_timer(duration, callback);
        RCLCPP_DEBUG(context.node->get_logger(), 
                    "Agent %d (%s): Created timer for %ld ms",
                    context.agent_id, getName().c_str(), duration.count());
    } else {
        RCLCPP_ERROR(context.node->get_logger(), 
                    "Agent %d (%s): Node is null, cannot create timer",
                    context.agent_id, getName().c_str());
    }
}

void MovementStrategy::stopAgent(const MovementContext& context)
{
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.linear.z = 0.0;
    stop_msg.angular.x = 0.0;
    stop_msg.angular.y = 0.0;
    stop_msg.angular.z = 0.0;
    
    publishTwist(context, stop_msg);
    
    RCLCPP_INFO(context.node->get_logger(), 
               "Agent %d (%s): Movement complete - agent stopped",
               context.agent_id, getName().c_str());
    
    // Call completion callback if provided
    if (context.completion_callback) {
        context.completion_callback();
    }
}

} // namespace strategies
} // namespace dirac_navigation
