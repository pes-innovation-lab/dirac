#ifndef DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <functional>
#include <chrono>

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Movement parameters for executing strategies
 */
struct MovementParameters
{
    double linear_speed = 2.0;      // Linear velocity (m/s)
    double angular_speed = 1.57;    // Angular velocity (rad/s)
    double move_distance = 1.0;     // Distance to move (units)
    double turn_angle = 1.57;       // Turn angle in radians (π/2 for 90°)
};

/**
 * @brief Context for movement execution
 */
struct MovementContext
{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Node::SharedPtr node;
    MovementParameters parameters;
    int32_t agent_id;
    std::function<void()> completion_callback;
};

/**
 * @brief Abstract base class for movement strategies
 */
class MovementStrategy
{
public:
    virtual ~MovementStrategy() = default;
    
    /**
     * @brief Execute the movement strategy
     * @param context Movement context containing all necessary information
     */
    virtual void execute(const MovementContext& context) = 0;
    
    /**
     * @brief Get the strategy name for logging purposes
     * @return Strategy name
     */
    virtual std::string getName() const = 0;

protected:
    /**
     * @brief Helper function to publish a twist message
     * @param context Movement context
     * @param twist Twist message to publish
     */
    void publishTwist(const MovementContext& context, const geometry_msgs::msg::Twist& twist);
    
    /**
     * @brief Helper function to create a timer for movement duration
     * @param context Movement context
     * @param duration Duration for the timer
     * @param callback Callback function to execute when timer expires
     */
    void createMovementTimer(const MovementContext& context, 
                           std::chrono::milliseconds duration,
                           std::function<void()> callback);
    
    /**
     * @brief Helper function to stop the agent
     * @param context Movement context
     */
    void stopAgent(const MovementContext& context);
};

} // namespace strategies
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_HPP_
