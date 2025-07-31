#pragma once

#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "dirac_msgs/msg/agent_command.hpp"
#include "dirac_msgs/msg/job_top.hpp"
#include "std_msgs/msg/int32.hpp"
#include "bt/agent_state_db.hpp"
#include "bt/state_updater.hpp"
#include "bt/big_tank.hpp"

namespace bt {

class PathPlanningHandler {
public:
    // Callback types for external communication
    using CommandPublishCallback = std::function<void(const dirac_msgs::msg::AgentCommand&)>;
    using LogCallback = std::function<void(const std::string&, const std::string&)>; // level, message
    
    // Configuration struct
    struct Config {
        int agent_id = 1;
        int zone_id = 1;
        std::string map_csv_path;
        std::string agents_csv_path;
        int tick_delay_seconds = 2;
        bool enable_debug_logging = false;
    };

    PathPlanningHandler(rclcpp::Node* node, const Config& config);
    ~PathPlanningHandler();

    // Main interface methods
    bool initialize();
    void shutdown();
    
    // Topic subscription handlers (to be called by external subscribers)
    void handle_job_top_message(const dirac_msgs::msg::JobTop::SharedPtr msg);
    void handle_zone_pop_message(const std_msgs::msg::Int32::SharedPtr msg);
    
    // Callbacks for external communication
    void set_command_publish_callback(CommandPublishCallback callback);
    void set_log_callback(LogCallback callback);
    
    // Status getters
    bool is_initialized() const { return initialized_; }
    bool has_received_job_data() const { return job_id_received_; }
    bool has_received_zone_data() const { return zone_pop_received_; }
    int get_agent_id() const { return agent_id_; }
    std::string get_job_id() const { return job_id_; }
    std::pair<int, int> get_current_position() const;
    std::pair<int, int> get_goal_position() const { return goal_; }
    bool is_goal_reached() const;
    
    // Manual control (for testing/debugging)
    void force_tick_processing();
    void reset_state();

private:
    // Internal initialization and processing
    void try_initialize();
    void initialize_agent_state();
    void on_tick_change(int new_tick);
    void process_tick(int tick);
    
    // Logging helper
    void log(const std::string& level, const std::string& message);

    // Configuration
    Config config_;
    rclcpp::Node* node_;
    
    // Core components
    std::shared_ptr<bt::AgentStateDB> db_;
    std::unique_ptr<bt::StateUpdater> state_updater_;
    
    // Agent state variables
    int agent_id_;
    int current_tick_;
    bool is_leader_;
    std::pair<int, int> start_;
    int priority_;
    std::string job_id_;
    std::pair<int, int> goal_;
    std::vector<std::vector<int>> map_;
    std::vector<std::pair<int, int>> ideal_path_;
    
    // Status flags
    int total_agents_ = 0;
    bool job_id_received_ = false;
    bool zone_pop_received_ = false;
    bool initialized_ = false;
    
    // Callbacks
    CommandPublishCallback command_publish_callback_;
    LogCallback log_callback_;
};

} // namespace bt
