#include "bt/big_tank.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace bt {

AgentState BigTank::calculate_next_move(const AgentState& current_state, 
                                       const std::vector<std::pair<int, int>>& ideal_path,
                                       const std::pair<int, int>& goal,
                                       const std::vector<std::vector<int>>& map) {
    
    AgentState new_state = current_state;
    
    // If goal already reached, agent stays put and doesn't move
    if (current_state.goal_reached) {
        new_state.is_moving = false;
        new_state.force.first = 0.0;
        new_state.force.second = 0.0;
        // Keep current position (no movement)
        return new_state;
    }
    
    // If no path available, can't move
    if (ideal_path.empty()) {
        new_state.is_moving = false;
        new_state.force.first = 0.0;
        new_state.force.second = 0.0;
        return new_state;
    }
    
    int current_x = current_state.current_x;
    int current_y = current_state.current_y;
    
    // Find next waypoint in A* path
    auto next_waypoint = find_next_waypoint(current_x, current_y, ideal_path, goal);
    
    // Calculate force vector to next waypoint
    auto force = calculate_force(current_x, current_y, 
                                next_waypoint.first, next_waypoint.second,
                                current_state.force_multiplier);
    
    // Store the force in agent state
    new_state.force.first = force.first;
    new_state.force.second = force.second;
    
    // Convert force to discrete movement
    auto new_position = force_to_movement(current_x, current_y, force.first, force.second);
    
    // Validate movement is valid
    if (is_valid_position(new_position.first, new_position.second, map)) {
        // Update position
        new_state.current_x = new_position.first;
        new_state.current_y = new_position.second;
        new_state.is_moving = true;
        
        // Add to registered path (next_moves vector)
        new_state.next_moves.push_back({new_position.first, new_position.second});
        
        // Check if goal reached
        if (new_position.first == goal.first && new_position.second == goal.second) {
            new_state.goal_reached = true;
            new_state.is_moving = false;
        }
    } else {
        // Can't move - obstacle or boundary
        new_state.is_moving = false;
        // Keep current position
        new_state.current_x = current_x;
        new_state.current_y = current_y;
    }
    
    return new_state;
}

std::pair<int, int> BigTank::find_next_waypoint(int current_x, int current_y,
                                               const std::vector<std::pair<int, int>>& ideal_path,
                                               const std::pair<int, int>& goal) {
    // Find current position in A* path, then return the next waypoint
    for (size_t i = 0; i < ideal_path.size(); ++i) {
        if (ideal_path[i].first == current_x && ideal_path[i].second == current_y) {
            // Found current position, return next waypoint if it exists
            if (i + 1 < ideal_path.size()) {
                auto next = ideal_path[i + 1];
                return next;
            } else {
                // Already at the last waypoint, go to goal
                return goal;
            }
        }
    }
    
    // Current position not in path, go directly to goal
    RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
        "Current position (%d,%d) NOT FOUND in ideal path! Going directly to goal (%d,%d)", 
        current_x, current_y, goal.first, goal.second);
    return goal;
}

std::pair<double, double> BigTank::calculate_force(int current_x, int current_y,
                                                   int target_x, int target_y,
                                                   double force_multiplier) {
    double force_x = target_x - current_x;
    double force_y = target_y - current_y;
    
    return {force_x * force_multiplier, force_y * force_multiplier};
}

std::pair<int, int> BigTank::force_to_movement(int current_x, int current_y,
                                              double force_x, double force_y) {
    int new_x = current_x;
    int new_y = current_y;
    
    if (std::abs(force_x) > std::abs(force_y)) {
        // Move horizontally
        new_x = current_x + (force_x > 0 ? 1 : -1);
    } else if (std::abs(force_y) > 0) {
        // Move vertically  
        new_y = current_y + (force_y > 0 ? 1 : -1);
    }
    
    return {new_x, new_y};
}

bool BigTank::is_valid_position(int x, int y, const std::vector<std::vector<int>>& map) {
    if (map.empty() || map[0].empty()) {
        return false;
    }
    
    return x >= 0 && x < (int)map[0].size() && 
           y >= 0 && y < (int)map.size() && 
           map[y][x] == 0;
}

} // namespace bt
