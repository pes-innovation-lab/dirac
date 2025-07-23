#ifndef DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_FACTORY_HPP_
#define DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_FACTORY_HPP_

#include "dirac_navigation/strategies/movement_strategy.hpp"
#include <memory>

namespace dirac_navigation
{
namespace strategies
{

/**
 * @brief Direction codes for movement commands
 */
enum class MovementDirection : int32_t
{
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
};

/**
 * @brief Factory for creating movement strategies
 */
class MovementStrategyFactory
{
public:
    /**
     * @brief Create a movement strategy based on direction
     * @param direction Direction code
     * @return Unique pointer to movement strategy, nullptr if invalid direction
     */
    static std::unique_ptr<MovementStrategy> createStrategy(MovementDirection direction);
    
    /**
     * @brief Create a movement strategy based on direction code
     * @param direction_code Integer direction code
     * @return Unique pointer to movement strategy, nullptr if invalid direction
     */
    static std::unique_ptr<MovementStrategy> createStrategy(int32_t direction_code);
    
    /**
     * @brief Validate if a direction code is valid
     * @param direction_code Integer direction code to validate
     * @return True if valid, false otherwise
     */
    static bool isValidDirection(int32_t direction_code);
    
    /**
     * @brief Convert direction code to MovementDirection enum
     * @param direction_code Integer direction code
     * @return MovementDirection enum value
     * @throws std::invalid_argument if direction code is invalid
     */
    static MovementDirection toMovementDirection(int32_t direction_code);
};

} // namespace strategies
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_STRATEGIES_MOVEMENT_STRATEGY_FACTORY_HPP_
