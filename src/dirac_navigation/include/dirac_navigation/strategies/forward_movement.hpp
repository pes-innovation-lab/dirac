#ifndef DIRAC_NAVIGATION_STRATEGIES_FORWARD_MOVEMENT_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_FORWARD_MOVEMENT_HPP_

#include "dirac_navigation/strategies/movement_strategy.hpp"

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Strategy for moving forward
 */
class ForwardMovement : public MovementStrategy
{
public:
    /**
     * @brief Execute forward movement
     * @param context Movement context containing all necessary information
     */
    void execute(const MovementContext& context) override;
    
    /**
     * @brief Get the strategy name
     * @return "ForwardMovement"
     */
    std::string getName() const override { return "ForwardMovement"; }
};

} // namespace strategies
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_STRATEGIES_FORWARD_MOVEMENT_HPP_
