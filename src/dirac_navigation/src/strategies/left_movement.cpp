#include "dirac_navigation/strategies/left_movement.hpp"

namespace dirac_navigation
{
namespace strategies
{

void LeftMovement::execute(const MovementContext& context)
{
    RCLCPP_INFO(context.node->get_logger(), 
               "Agent %d: Executing left movement (turn-move-turn sequence)",
               context.agent_id);
    
    // Step 1: Turn left 90 degrees
    executeTurn(context, context.parameters.angular_speed, [this, context]() {
        // Step 2: Move forward the specified distance
        executeForward(context, [this, context]() {
            // Step 3: Turn right 90 degrees to restore original orientation
            executeTurn(context, -context.parameters.angular_speed, [this, context]() {
                stopAgent(context);
            });
        });
    });
}

void LeftMovement::executeTurn(const MovementContext& context, 
                              double angular_velocity, 
                              std::function<void()> callback)
{
    // Create and publish turn command
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = angular_velocity;
    
    publishTwist(context, twist_msg);
    
    // Calculate duration to turn the specified angle
    auto duration = std::chrono::milliseconds(
        static_cast<int>((context.parameters.turn_angle / std::abs(angular_velocity)) * 1000)
    );
    
    // Create timer to proceed to next step after turn completes
    createMovementTimer(context, duration, callback);
}

void LeftMovement::executeForward(const MovementContext& context, 
                                 std::function<void()> callback)
{
    // Create and publish forward movement command
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = context.parameters.linear_speed;
    twist_msg.angular.z = 0.0;
    
    publishTwist(context, twist_msg);
    
    // Calculate duration to move the specified distance
    auto duration = std::chrono::milliseconds(
        static_cast<int>((context.parameters.move_distance / context.parameters.linear_speed) * 1000)
    );
    
    // Create timer to proceed to next step after forward movement completes
    createMovementTimer(context, duration, callback);
}

} // namespace strategies
} // namespace dirac_navigation
