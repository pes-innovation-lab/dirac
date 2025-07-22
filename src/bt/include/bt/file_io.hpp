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
    // Given filename and agent_id, returns ((start_x, start_y), job_id, (goal_x, goal_y)) for that agent
    static std::tuple<std::pair<int, int>, std::string, std::pair<int, int>>
    get_agent_info(const std::string& filename, int agent_id) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open agents file: " << filename << std::endl;
            return {{-1, -1}, "", {-1, -1}};
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
                std::getline(ss, token, ','); start_x = std::stoi(token);
                std::getline(ss, token, ','); start_y = std::stoi(token);
                std::getline(ss, token, ','); priority = std::stoi(token); // skip
                std::getline(ss, token, ','); job_id = token;
                std::getline(ss, token, ','); goal_x = std::stoi(token);
                std::getline(ss, token, ','); goal_y = std::stoi(token);
                return {{start_x, start_y}, job_id, {goal_x, goal_y}};
            }
        }
        return {{-1, -1}, "", {-1, -1}}; // not found
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
