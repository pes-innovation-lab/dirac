#include <string>
#include <fstream>
#include <functional>
#include <filesystem>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bt/file_io.hpp"
#include "bt/agent_state_db.hpp"
#include "bt/state_updater.hpp"
#include "bt/path_planner.hpp"
#include "bt/big_tank.hpp"

class Agent : public rclcpp::Node {
public:
    Agent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("agent", options) {
        this->declare_parameter<int>("agent_id", 1);
        
        agent_id_ = this->get_parameter("agent_id").as_int();

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("bt");
        std::string agents_csv_path = package_share_directory + "/agents.csv";
        std::string map_csv_path = package_share_directory + "/map.csv";

        RCLCPP_INFO(this->get_logger(), "Looking for agents.csv at: %s", agents_csv_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Looking for map.csv at: %s", map_csv_path.c_str());

        auto [start, job_id, goal, is_leader] = bt::FileIO::get_agent_info(agents_csv_path, agent_id_);
        start_ = start;
        job_id_ = job_id;
        goal_ = goal;
        is_leader_ = is_leader;
        map_ = bt::FileIO::read_map_csv(map_csv_path);
        ideal_path_ = bt::PathPlanner::astar_path(start_.first, start_.second, goal_.first, goal_.second, map_);
        db_ = std::make_shared<bt::AgentStateDB>();

        // Count total agents from agents.csv
        int total_agents = 0;
        {
            std::ifstream infile(agents_csv_path);
            std::string line;
            // Skip header
            std::getline(infile, line);
            while (std::getline(infile, line)) {
                if (!line.empty()) ++total_agents;
            }
        }
        
        // Initialize agent state in database FIRST
        initialize_agent_state();

        // Create StateUpdater AFTER agent state is initialized
        state_updater_ = std::make_unique<bt::StateUpdater>(this, agent_id_, db_, total_agents, agents_csv_path);

        // Set up tick change callback
        state_updater_->set_tick_change_callback([this](int new_tick) {
            this->on_tick_change(new_tick);
        });

        RCLCPP_INFO(this->get_logger(), "Agent %d initialized with leadership: %s", 
                    agent_id_, is_leader_ ? "YES (LEADER)" : "NO");

        // Start with tick 0 - leader will advance when all agents are ready
        current_tick_ = 0;
        if (state_updater_->get_current_global_tick() == 0) {
            RCLCPP_INFO(this->get_logger(), "Agent %d starting initial tick processing", agent_id_);
            // Kick off the first tick by publishing initial state
            process_tick(current_tick_);
        }

        RCLCPP_INFO(this->get_logger(), "Agent %d start: (%d,%d), job_id: %s, goal: (%d,%d)", agent_id_, start_.first, start_.second, job_id_.c_str(), goal_.first, goal_.second);
        RCLCPP_INFO(this->get_logger(), "Loaded map of size %lux%lu", map_.size(), map_.empty() ? 0 : map_[0].size());
        RCLCPP_INFO(this->get_logger(), "Ideal path length: %lu", ideal_path_.size());
    }

private:
    void initialize_agent_state() {
        AgentState initial_state;
        initial_state.agent_id = agent_id_;
        initial_state.current_x = start_.first;
        initial_state.current_y = start_.second;
        initial_state.priority = 1; // Default priority
        initial_state.job_id = job_id_;
        initial_state.goal_x = goal_.first;
        initial_state.goal_y = goal_.second;
        initial_state.is_moving = false;
        initial_state.is_leader = is_leader_; // Use the value from CSV
        initial_state.current_tick = 0;
        initial_state.goal_reached = false;
        initial_state.force = {0.0, 0.0};
        initial_state.chain_force = {0.0, 0.0};
        initial_state.stuck_counter = 0;
        initial_state.force_multiplier = 1.0;
        initial_state.timestamp = std::chrono::system_clock::now();
        
        db_->setState(agent_id_, initial_state);
        RCLCPP_INFO(this->get_logger(), "Agent %d state initialized in database", agent_id_);
    }

    void on_tick_change(int new_tick) {
        if (new_tick > current_tick_) {
            current_tick_ = new_tick;
            RCLCPP_INFO(this->get_logger(), "Agent %d received tick change to %d", agent_id_, current_tick_);
            process_tick(current_tick_);
        }
    }

    void process_tick(int tick) {
        RCLCPP_INFO(this->get_logger(), "Agent %d starting processing for tick %d", agent_id_, tick);
        
        // Add delay to slow down tick processing for observation
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        AgentState current_state = db_->getState(agent_id_);
        current_state.current_tick = tick;
        current_state.timestamp = std::chrono::system_clock::now();
        
        // Use BigTank algorithm to calculate next move
        AgentState new_state = bt::BigTank::calculate_next_move(current_state, ideal_path_, goal_, map_);
        
        // Log movement information
        if (new_state.current_x != current_state.current_x || new_state.current_y != current_state.current_y) {
            RCLCPP_INFO(this->get_logger(), "Agent %d moved from (%d,%d) to (%d,%d) with force (%.2f,%.2f)", 
                       agent_id_, current_state.current_x, current_state.current_y, 
                       new_state.current_x, new_state.current_y, 
                       new_state.force.first, new_state.force.second);
        } else if (!new_state.is_moving && !new_state.goal_reached) {
            RCLCPP_WARN(this->get_logger(), "Agent %d blocked at (%d,%d), cannot move", 
                       agent_id_, current_state.current_x, current_state.current_y);
        }
        
        // Check if goal reached
        if (new_state.goal_reached) {
            RCLCPP_INFO(this->get_logger(), "Agent %d reached goal at (%d,%d)!", 
                       agent_id_, new_state.current_x, new_state.current_y);
        }
        
        // Update state in database
        db_->setState(agent_id_, new_state);
        
        // Trigger distributed coordination by publishing state
        state_updater_->publish_state_for_tick(tick);
        
        RCLCPP_INFO(this->get_logger(), "Agent %d completed processing tick %d", agent_id_, tick);
    }

private:
    int agent_id_;
    int current_tick_;
    bool is_leader_;
    std::pair<int, int> start_;
    std::string job_id_;
    std::pair<int, int> goal_;
    std::vector<std::vector<int>> map_;
    std::vector<std::pair<int, int>> ideal_path_;
    std::shared_ptr<bt::AgentStateDB> db_;
    std::unique_ptr<bt::StateUpdater> state_updater_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Agent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
