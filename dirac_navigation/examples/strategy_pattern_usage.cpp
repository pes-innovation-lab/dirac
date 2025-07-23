// Example usage of the new Strategy Pattern implementation
// This file demonstrates how the NavigationController now uses strategies

#include "dirac_navigation/navigation_controller.hpp"
#include "dirac_navigation/strategies/movement_strategy_factory.hpp"

/*
Example of how the refactored NavigationController works:

// Old way (removed):
navigation_controller->move_forward(agent_id);
navigation_controller->move_left(agent_id);

// New way (strategy pattern):
navigation_controller->execute_command(agent_id, 1);  // Forward
navigation_controller->execute_command(agent_id, 3);  // Left

// The NavigationController now:
// 1. Creates a strategy using MovementStrategyFactory::createStrategy(direction)
// 2. Creates a MovementContext with all necessary information
// 3. Executes the strategy with strategy->execute(context)

// Benefits of the strategy pattern refactoring:
// - Cleaner separation of concerns
// - Each movement type is in its own class
// - Easy to add new movement patterns
// - Better testability (can test each strategy independently)
// - Consistent interface for all movements
// - Centralized movement parameter management

// To add a new movement strategy:
// 1. Create a new class inheriting from MovementStrategy
// 2. Implement execute() and getName() methods
// 3. Add the strategy to MovementStrategyFactory
// 4. Add the direction enum to MovementDirection

// Example of adding a diagonal movement:
class DiagonalMovement : public MovementStrategy {
public:
    void execute(const MovementContext& context) override {
        // Implementation for diagonal movement
    }
    std::string getName() const override { return "DiagonalMovement"; }
};
*/
