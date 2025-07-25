#ifndef DIRAC_NAVIGATION_STRATEGIES_RIGHT_MOVEMENT_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_RIGHT_MOVEMENT_HPP_

#include "dirac_navigation/strategies/movement_strategy.hpp"

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Strategy for moving right (turn-move-turn sequence)
 */
class RightMovement : public MovementStrategy
{
public:
    /**
     * @brief Execute right movement (turn right, move forward, turn left)
     * @param context Movement context containing all necessary information
     */
    void execute(const MovementContext& context) override;
    
    /**
     * @brief Get the strategy name
     * @return "RightMovement"
     */
    std::string getName() const override { return "RightMovement"; }

private:
    /**
     * @brief Execute the turn phase of the movement
     * @param context Movement context
     * @param angular_velocity Angular velocity for the turn
     * @param callback Callback to execute after turn completes
     */
    void executeTurn(const MovementContext& context, 
                    double angular_velocity, 
                    std::function<void()> callback);
    
    /**
     * @brief Execute the forward movement phase
     * @param context Movement context
     * @param callback Callback to execute after movement completes
     */
    void executeForward(const MovementContext& context, 
                       std::function<void()> callback);
};

} // namespace strategies
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_STRATEGIES_RIGHT_MOVEMENT_HPP_
