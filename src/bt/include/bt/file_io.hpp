#pragma once 

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "bt/agent_state.hpp"

namespace bt {

class FileIO {
public:
    // Reads agents.csv and returns a vector of AgentState (with only fields from CSV filled)
    static std::vector<AgentState> read_agents_csv(const std::string& filename) {
        std::vector<AgentState> agents;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open agents file: " << filename << std::endl;
            return agents;
        }
        std::string line;
        std::getline(file, line); // skip header
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            AgentState agent;
            std::string token;
            std::getline(ss, token, ','); agent.agent_id = std::stoi(token);
            std::getline(ss, token, ','); agent.current_x = std::stoi(token);
            std::getline(ss, token, ','); agent.current_y = std::stoi(token);
            std::getline(ss, token, ','); agent.priority = std::stoi(token);
            std::getline(ss, token, ','); agent.job_id = token;
            std::getline(ss, token, ','); agent.goal_x = std::stoi(token);
            std::getline(ss, token, ','); agent.goal_y = std::stoi(token);
            agent.current_tick = 0;
            agent.goal_reached = false;
            agent.force = {0.0, 0.0};
            agent.chain_force = {0.0, 0.0};
            agent.stuck_counter = 0;
            agent.force_multiplier = static_cast<double>(agent.priority);
            agent.timestamp = std::chrono::system_clock::now();
            agents.push_back(agent);
        }
        return agents;
    }

    // Reads map.csv and returns a 2D vector of ints
    static std::vector<std::vector<int>> read_map_csv(const std::string& filename) {
        std::vector<std::vector<int>> map;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open map file: " << filename << std::endl;
            return map;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::vector<int> row;
            std::stringstream ss(line);
            std::string token;
            while (std::getline(ss, token, ',')) {
                row.push_back(std::stoi(token));
            }
            map.push_back(row);
        }
        return map;
    }
};

} // namespace bt
