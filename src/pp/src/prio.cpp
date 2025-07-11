#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"


#include <string>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <vector>
#include <algorithm>
#include <sstream>
#include <utility>
#include <chrono>


struct agent {
    int agent_id;
    int agent_x;
    int agent_y;
    int priority;
    std::string job_id;
    int job_x;
    int job_y;
    std::vector<std::string> ideal_path;
    std::vector<std::pair<int, int>> registered_path = {};
};

#define MAP_SIZE 20
struct cell {
    int x;
    int y;
    struct agent* agent; 
    double f; // total cost
    double g; // cost from start to current cell
    double h; // heuristic cost to goal
};

std::vector<cell> occupied_now; // Global vector to store occupied cells

// 20x20 warehouse-style map
int map_[20][20] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

std::vector<agent> agents = {
    {
        1, 0, 0, 1, "J1", 19, 2,
        {
            "{0,0}","{1,0}","{2,0}","{3,0}","{4,0}",
            "{5,0}","{6,0}","{7,0}","{8,0}","{9,0}",
            "{10,0}","{11,0}","{12,0}","{13,0}","{14,0}",
            "{15,0}","{16,0}","{17,0}","{18,0}","{19,0}",
            "{19,1}","{19,2}"   
        }
    },
    {
        2, 0, 4, 2, "J2", 19, 6,
        {
            "{0,4}","{1,4}","{2,4}","{3,4}","{4,4}",
            "{5,4}","{6,4}","{7,4}","{8,4}","{9,4}",
            "{10,4}","{11,4}","{12,4}","{13,4}","{14,4}",
            "{15,4}","{16,4}","{17,4}","{18,4}","{19,4}",
            "{19,5}","{19,6}"
        }
    },
    {
        3, 0, 8, 3, "J3", 19, 10,
        {
            "{0,8}","{1,8}","{2,8}","{3,8}","{4,8}",
            "{5,8}","{6,8}","{7,8}","{8,8}","{9,8}",
            "{10,8}","{11,8}","{12,8}","{13,8}","{14,8}",
            "{15,8}","{16,8}","{17,8}","{18,8}","{19,8}",
            "{19,9}","{19,10}"
        }
    },
    {
        4, 0, 12, 4, "J4", 19, 14,
        {
            "{0,12}","{1,12}","{2,12}","{3,12}","{4,12}",
            "{5,12}","{6,12}","{7,12}","{8,12}","{9,12}",
            "{10,12}","{11,12}","{12,12}","{13,12}","{14,12}",
            "{15,12}","{16,12}","{17,12}","{18,12}","{19,12}",
            "{19,13}","{19,14}"
        }
    },
    {
        5, 0, 16, 5, "J5", 19, 18,
        {
            "{0,16}","{1,16}","{2,16}","{3,16}","{4,16}",
            "{5,16}","{6,16}","{7,16}","{8,16}","{9,16}",
            "{10,16}","{11,16}","{12,16}","{13,16}","{14,16}",
            "{15,16}","{16,16}","{17,16}","{18,16}","{19,16}",
            "{19,17}","{19,18}"
        }
    }
};


class AgentNode : public rclcpp::Node {
public:
    AgentNode() : Node("agent_node"), timestep_(0) {
        declare_parameter<int>("agent_id", 1);
        agent_id_ = get_parameter("agent_id").as_int();

        agent_ = getAgentById(agent_id_);
        agent_path_ = getPath(agent_.ideal_path);
        RCLCPP_INFO(get_logger(), "Agent ID: %d, Job ID: %s, Path Size: %zu",
                    agent_.agent_id, agent_.job_id.c_str(), agent_.ideal_path.size());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&AgentNode::timerCallback, this)
        );
    }

private:
    agent getAgentById(int agent_id) {
        for (const auto& ag : agents) {
            if (ag.agent_id == agent_id) {
                return ag;
            }
        }
        RCLCPP_ERROR(get_logger(), "Agent with ID %d not found", agent_id);
        return {};
    }
    std::vector<cell> getOccupiedCells(int timestep){
        std::vector<cell> occupied_cells;
        cell occupied_cell;
        for(int i=0; i<static_cast<int>(agents.size()); i++){
            std::vector<std::pair<int,int>> ideal_path = getPath(agents[i].ideal_path);
            if (static_cast<int>(agents[i].ideal_path.size()) > timestep){
                occupied_cell.x = ideal_path[timestep].first;
                occupied_cell.y = ideal_path[timestep].second;
                occupied_cell.agent = &agents[i];
                occupied_cell.f = 0; // Placeholder, can be calculated if needed
                occupied_cell.g = 0; // Placeholder, can be calculated if needed        
                occupied_cell.h = 0; // Placeholder, can be calculated if needed
                occupied_cells.push_back(occupied_cell);
            }   
        }
        return occupied_cells;
    }
    // Callback for periodic update
    void timerCallback() {
        if (timestep_ >= static_cast<int>(agent_path_.size())) {
            RCLCPP_INFO(get_logger(), "Agent %d has completed its path.", agent_id_);
            return;
        }

        std::vector<cell> occupied_cells = getOccupiedCells(timestep_);
        occupied_now = occupied_cells;
        auto current_pos = agent_path_[timestep_];

        bool is_free = std::none_of(occupied_cells.begin(), occupied_cells.end(),
            [&](const cell& c) {
                return c.x == current_pos.first && c.y == current_pos.second;
            });

        if (is_free) {
            agent_.registered_path.push_back(current_pos);
            RCLCPP_INFO(get_logger(), "Timestep %d: Agent %d moving to (%d, %d)",
                        timestep_, agent_id_, current_pos.first, current_pos.second);
            timestep_++;
        } else {
            // Find the colliding agent
            auto it = std::find_if(occupied_cells.begin(), occupied_cells.end(),
                [&](const cell& c) {
                    return c.x == current_pos.first && c.y == current_pos.second;
                });

            if (it != occupied_cells.end()) {
                agent* conflicting_agent = it->agent;

                RCLCPP_WARN(get_logger(), 
                    "Timestep %d: Agent %d collision at (%d, %d) with Agent %d",
                    timestep_, agent_id_, current_pos.first, current_pos.second,
                    conflicting_agent->agent_id);

                // TODO: Collision resolution logic based on agent priority
                if( agent_.priority < conflicting_agent->priority) {
                    RCLCPP_INFO(get_logger(), "Agent %d will wait due to lower priority.", agent_id_);
                } else {
                    // Higher priority agent logic can be added here
                    RCLCPP_INFO(get_logger(), "Agent %d has higher priority, proceeding.", agent_id_);
                    agent_.registered_path.push_back(current_pos);
                    timestep_++;
                }
            }
        }

    }


    std::vector<std::pair<int, int>> getPath(const std::vector<std::string>& path_strs) {
        std::vector<std::pair<int, int>> result;
        for (const auto& item : path_strs) {
            size_t start = item.find('{');
            size_t end = item.find('}');
            if (start != std::string::npos && end != std::string::npos) {
                std::string coords = item.substr(start + 1, end - start - 1);
                size_t comma = coords.find(',');
                if (comma != std::string::npos) {
                    int x = std::stoi(coords.substr(0, comma));
                    int y = std::stoi(coords.substr(comma + 1));
                    result.emplace_back(x, y);
                }
            }
        }
        return result;
    }

    agent agent_;
    int agent_id_;
    int timestep_;
    std::vector<std::pair<int, int>> agent_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}