#pragma once
#include <vector>
#include <utility>

namespace bt {

class PathPlanner {
public:
    // Calculates A* path from (start_x, start_y) to (goal_x, goal_y) on the given map
    // Returns a vector of (x, y) pairs representing the path (including start and goal)
    static std::vector<std::pair<int, int>> astar_path(
        int start_x, int start_y,
        int goal_x, int goal_y,
        const std::vector<std::vector<int>>& map);
};

} // namespace bt
