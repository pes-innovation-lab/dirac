#pragma once

#include <vector>
#include <utility>
#include "bt/agent_state.hpp"

namespace bt {

class BigTank {
public:
    /**
     * Calculate the next position for an agent using Big Tank force-based movement
     * @param current_state Current agent state
     * @param ideal_path A* path from start to goal
     * @param goal Target position
     * @param map Grid map (0 = free, 1 = obstacle)
     * @return Updated agent state with new position and force
     */
    static AgentState calculate_next_move(const AgentState& current_state, 
                                         const std::vector<std::pair<int, int>>& ideal_path,
                                         const std::pair<int, int>& goal,
                                         const std::vector<std::vector<int>>& map);

private:
    /**
     * Find the next waypoint in the A* path from current position
     */
    static std::pair<int, int> find_next_waypoint(int current_x, int current_y,
                                                   const std::vector<std::pair<int, int>>& ideal_path,
                                                   const std::pair<int, int>& goal);
    
    /**
     * Calculate force vector from current position to target
     */
    static std::pair<double, double> calculate_force(int current_x, int current_y,
                                                     int target_x, int target_y,
                                                     double force_multiplier);
    
    /**
     * Convert force vector to discrete movement
     */
    static std::pair<int, int> force_to_movement(int current_x, int current_y,
                                                  double force_x, double force_y);
    
    /**
     * Check if a position is valid (within bounds and not an obstacle)
     */
    static bool is_valid_position(int x, int y, const std::vector<std::vector<int>>& map);
};

} // namespace bt
