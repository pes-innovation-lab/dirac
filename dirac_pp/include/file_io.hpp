#pragma once 

#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <sstream>
#include <iostream>
#include "bt/agent_state.hpp"

namespace bt {

class FileIO {
public:
    // Given filename and agent_id, returns ((start_x, start_y), priority, job_id, (goal_x, goal_y), is_leader) for that agent
    static std::tuple<std::pair<int, int>, int, std::string, std::pair<int, int>, bool>
    get_agent_info(const std::string& filename, int agent_id) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open agents file: " << filename << std::endl;
            return {{-1, -1}, 1, "", {-1, -1}, false};
        }
        std::string line;
        std::getline(file, line); // skip header
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            int id;
            std::getline(ss, token, ','); id = std::stoi(token);
            if (id == agent_id) {
                int start_x, start_y, priority, goal_x, goal_y;
                std::string job_id;
                bool is_leader;
                std::getline(ss, token, ','); start_x = std::stoi(token);
                std::getline(ss, token, ','); start_y = std::stoi(token);
                std::getline(ss, token, ','); priority = std::stoi(token);
                std::getline(ss, token, ','); job_id = token;
                std::getline(ss, token, ','); goal_x = std::stoi(token);
                std::getline(ss, token, ','); goal_y = std::stoi(token);
                std::getline(ss, token, ','); is_leader = (token == "true" || token == "1");
                return {{start_x, start_y}, priority, job_id, {goal_x, goal_y}, is_leader};
            }
        }
        return {{-1, -1}, 1, "", {-1, -1}, false}; // not found
    }

    // Get all agent IDs from the CSV file
    static std::set<int> get_all_agent_ids(const std::string& filename) {
        std::set<int> agent_ids;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open agents file: " << filename << std::endl;
            return agent_ids;
        }
        std::string line;
        std::getline(file, line); // skip header
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::getline(ss, token, ',');
            agent_ids.insert(std::stoi(token));
        }
        return agent_ids;
    }

    // Get complete agent data for all agents
    static std::vector<AgentState> get_all_agent_data(const std::string& filename) {
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
            std::string token;
            AgentState agent;
            
            std::getline(ss, token, ','); agent.agent_id = std::stoi(token);
            std::getline(ss, token, ','); agent.current_x = std::stoi(token);
            std::getline(ss, token, ','); agent.current_y = std::stoi(token);
            std::getline(ss, token, ','); agent.priority = std::stoi(token);
            std::getline(ss, token, ','); agent.job_id = token;
            std::getline(ss, token, ','); agent.goal_x = std::stoi(token);
            std::getline(ss, token, ','); agent.goal_y = std::stoi(token);
            std::getline(ss, token, ','); agent.is_leader = (token == "true" || token == "1");
            
            // Set defaults for other fields
            agent.is_moving = false;
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
