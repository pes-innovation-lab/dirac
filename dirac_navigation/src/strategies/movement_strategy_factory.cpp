#include "dirac_navigation/strategies/movement_strategy_factory.hpp"
#include "dirac_navigation/strategies/forward_movement.hpp"
#include "dirac_navigation/strategies/backward_movement.hpp"
#include "dirac_navigation/strategies/left_movement.hpp"
#include "dirac_navigation/strategies/right_movement.hpp"
#include <stdexcept>

namespace dirac_navigation
{
namespace strategies
{

std::unique_ptr<MovementStrategy> MovementStrategyFactory::createStrategy(MovementDirection direction)
{
    switch (direction) {
        case MovementDirection::FORWARD:
            return std::make_unique<ForwardMovement>();
        case MovementDirection::BACKWARD:
            return std::make_unique<BackwardMovement>();
        case MovementDirection::LEFT:
            return std::make_unique<LeftMovement>();
        case MovementDirection::RIGHT:
            return std::make_unique<RightMovement>();
        default:
            return nullptr;
    }
}

std::unique_ptr<MovementStrategy> MovementStrategyFactory::createStrategy(int32_t direction_code)
{
    if (!isValidDirection(direction_code)) {
        return nullptr;
    }
    
    try {
        MovementDirection direction = toMovementDirection(direction_code);
        return createStrategy(direction);
    } catch (const std::invalid_argument&) {
        return nullptr;
    }
}

bool MovementStrategyFactory::isValidDirection(int32_t direction_code)
{
    return direction_code >= static_cast<int32_t>(MovementDirection::FORWARD) && 
           direction_code <= static_cast<int32_t>(MovementDirection::RIGHT);
}

MovementDirection MovementStrategyFactory::toMovementDirection(int32_t direction_code)
{
    if (!isValidDirection(direction_code)) {
        throw std::invalid_argument("Invalid direction code: " + std::to_string(direction_code));
    }
    
    return static_cast<MovementDirection>(direction_code);
}

} // namespace strategies
} // namespace dirac_navigation
