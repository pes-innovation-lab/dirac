#pragma once
#include <string>
#include <vector>
#include <utility>
#include <chrono>

struct AgentState {
    int agent_id;
    int current_x, current_y;
    int priority;
    std::string job_id;
    int goal_x, goal_y;
    bool is_moving;
    bool is_leader;
    int current_tick;
    std::vector<std::pair<int, int>> next_moves;
    bool goal_reached;
    std::pair<double, double> force;
    std::pair<double, double> chain_force; 
    int stuck_counter = 0;                 
    double force_multiplier = static_cast<double>(priority); 
    std::chrono::system_clock::time_point timestamp;
};
