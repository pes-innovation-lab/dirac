#ifndef DIRAC_NAVIGATION_STRATEGIES_LEFT_MOVEMENT_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_LEFT_MOVEMENT_HPP_

#include "dirac_navigation/strategies/movement_strategy.hpp"

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Strategy for moving left (turn-move-turn sequence)
 */
class LeftMovement : public MovementStrategy
{
public:
    /**
     * @brief Execute left movement (turn left, move forward, turn right)
     * @param context Movement context containing all necessary information
     */
    void execute(const MovementContext& context) override;
    
    /**
     * @brief Get the strategy name
     * @return "LeftMovement"
     */
    std::string getName() const override { return "LeftMovement"; }

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

#endif // DIRAC_NAVIGATION_STRATEGIES_LEFT_MOVEMENT_HPP_
