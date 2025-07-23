#ifndef DIRAC_NAVIGATION_STRATEGIES_BACKWARD_MOVEMENT_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_BACKWARD_MOVEMENT_HPP_

#include "dirac_navigation/strategies/movement_strategy.hpp"

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Strategy for moving backward
 */
class BackwardMovement : public MovementStrategy
{
public:
    /**
     * @brief Execute backward movement
     * @param context Movement context containing all necessary information
     */
    void execute(const MovementContext& context) override;
    
    /**
     * @brief Get the strategy name
     * @return "BackwardMovement"
     */
    std::string getName() const override { return "BackwardMovement"; }
};

} // namespace strategies
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_STRATEGIES_BACKWARD_MOVEMENT_HPP_
