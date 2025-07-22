#include "bt/path_planner.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace bt {

struct Node {
    int x, y;
    double g, h;
    Node* parent;
    Node(int x, int y, double g, double h, Node* parent=nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    double f() const { return g + h; }
};

struct NodeCmp {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

std::vector<std::pair<int, int>> PathPlanner::astar_path(
    int start_x, int start_y,
    int goal_x, int goal_y,
    const std::vector<std::vector<int>>& map)
{
    int rows = map.size();
    int cols = map.empty() ? 0 : map[0].size();
    auto in_bounds = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < cols && y < rows;
    };
    auto is_free = [&](int x, int y) {
        return in_bounds(x, y) && map[y][x] == 0;
    };
    auto heuristic = [&](int x, int y) {
        return std::abs(x - goal_x) + std::abs(y - goal_y);
    };
    std::priority_queue<Node*, std::vector<Node*>, NodeCmp> open;
    std::unordered_map<int, Node*> all_nodes;
    auto key = [&](int x, int y) { return y * cols + x; };
    Node* start = new Node(start_x, start_y, 0, heuristic(start_x, start_y));
    open.push(start);
    all_nodes[key(start_x, start_y)] = start;
    std::unordered_map<int, double> g_score;
    g_score[key(start_x, start_y)] = 0;
    std::vector<std::pair<int, int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};
    Node* goal_node = nullptr;
    while (!open.empty()) {
        Node* curr = open.top(); open.pop();
        if (curr->x == goal_x && curr->y == goal_y) {
            goal_node = curr;
            break;
        }
        for (auto [dx, dy] : directions) {
            int nx = curr->x + dx, ny = curr->y + dy;
            if (!is_free(nx, ny)) continue;
            double tentative_g = curr->g + 1;
            int nkey = key(nx, ny);
            if (!g_score.count(nkey) || tentative_g < g_score[nkey]) {
                g_score[nkey] = tentative_g;
                Node* neighbor = new Node(nx, ny, tentative_g, heuristic(nx, ny), curr);
                open.push(neighbor);
                all_nodes[nkey] = neighbor;
            }
        }
    }
    std::vector<std::pair<int, int>> path;
    if (goal_node) {
        for (Node* n = goal_node; n; n = n->parent)
            path.emplace_back(n->x, n->y);
        std::reverse(path.begin(), path.end());
    }
    for (auto& [_, n] : all_nodes) delete n;
    return path;
}

} // namespace bt
