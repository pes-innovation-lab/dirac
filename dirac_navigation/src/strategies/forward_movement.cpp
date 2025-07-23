#include "dirac_navigation/strategies/forward_movement.hpp"

namespace dirac_navigation
{
namespace strategies
{

void ForwardMovement::execute(const MovementContext& context)
{
    RCLCPP_INFO(context.node->get_logger(), 
               "Agent %d: Executing forward movement (%.2f units at %.2f m/s)",
               context.agent_id, context.parameters.move_distance, context.parameters.linear_speed);
    
    // Create and publish forward movement command
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = context.parameters.linear_speed;
    twist_msg.angular.z = 0.0;
    
    publishTwist(context, twist_msg);
    
    // Calculate duration to move the specified distance
    auto duration = std::chrono::milliseconds(
        static_cast<int>((context.parameters.move_distance / context.parameters.linear_speed) * 1000)
    );
    
    // Create timer to stop movement after the calculated duration
    createMovementTimer(context, duration, [this, context]() {
        stopAgent(context);
    });
}

} // namespace strategies
} // namespace dirac_navigation
