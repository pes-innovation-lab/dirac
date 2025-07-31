#include "bt/path_planning_handler.hpp"
#include "bt/file_io.hpp"
#include "bt/path_planner.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <thread>
#include <chrono>

namespace bt {

PathPlanningHandler::PathPlanningHandler(rclcpp::Node* node, const Config& config)
    : config_(config)
    , node_(node)
    , agent_id_(config.agent_id)
    , current_tick_(0)
    , is_leader_(false)
    , start_({0, 0})
    , priority_(1)
    , goal_({0, 0})
{
    // Initialize database
    db_ = std::make_shared<bt::AgentStateDB>();
    
    log("INFO", "PathPlanningHandler created for agent " + std::to_string(agent_id_));
}

PathPlanningHandler::~PathPlanningHandler() {
    shutdown();
}

bool PathPlanningHandler::initialize() {
    try {
        // Load map data
        std::string map_path = config_.map_csv_path;
        if (map_path.empty()) {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("bt");
            map_path = package_share_directory + "/map.csv";
        }
        log("INFO", "Loading map from: " + map_path);
        map_ = bt::FileIO::read_map_csv(map_path);
        if (map_.empty()) {
            log("ERROR", "Failed to load map from " + map_path);
            return false;
        }
        log("INFO", "Map loaded successfully. Size: " + std::to_string(map_.size()) + "x" + 
            std::to_string(map_.empty() ? 0 : map_[0].size()));

        // Load all agent start locations from agents.csv
        std::string agents_path = config_.agents_csv_path;
        if (agents_path.empty()) {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("bt");
            agents_path = package_share_directory + "/agents.csv";
        }
        log("INFO", "Loading agent start locations from: " + agents_path);
        std::ifstream infile(agents_path);
        if (!infile.is_open()) {
            log("ERROR", "Failed to open agents.csv at " + agents_path);
            return false;
        }
        std::string line;
        // Skip header
        std::getline(infile, line);
        while (std::getline(infile, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            if (tokens.size() < 3) continue; // Expect at least: id, start_x, start_y
            int id = std::stoi(tokens[0]);
            int start_x = std::stoi(tokens[1]);
            int start_y = std::stoi(tokens[2]);
            int priority = id; // Default: priority = agent_id
            AgentState state;
            state.agent_id = id;
            state.current_x = start_x;
            state.current_y = start_y;
            state.priority = priority;
            state.job_id = "";
            state.goal_x = 0;
            state.goal_y = 0;
            state.is_moving = false;
            state.is_leader = false;
            state.current_tick = 0;
            state.goal_reached = false;
            state.force = {0.0, 0.0};
            state.chain_force = {0.0, 0.0};
            state.stuck_counter = 0;
            state.force_multiplier = 1.0;
            state.timestamp = std::chrono::system_clock::now();
            state.next_moves.clear();
            db_->setState(id, state);
            log("INFO", "Initialized AgentState for agent " + std::to_string(id) + " at (" + std::to_string(start_x) + "," + std::to_string(start_y) + ") with priority " + std::to_string(priority));
        }
        infile.close();
        return true;
    } catch (const std::exception& e) {
        log("ERROR", "Failed to initialize PathPlanningHandler: " + std::string(e.what()));
        return false;
    }
}

void PathPlanningHandler::shutdown() {
    if (state_updater_) {
        state_updater_.reset();
    }
    initialized_ = false;
    log("INFO", "PathPlanningHandler shut down for agent " + std::to_string(agent_id_));
}

void PathPlanningHandler::handle_job_top_message(const dirac_msgs::msg::JobTop::SharedPtr msg) {
    agent_id_ = msg->agent_id;
    start_ = std::make_pair(msg->start_x, msg->start_y);
    priority_ = msg->priority;
    job_id_ = msg->job_id;
    goal_ = std::make_pair(msg->goal_x, msg->goal_y);
    is_leader_ = msg->is_leader;
    
    // Recompute ideal_path with new start/goal
    if (!map_.empty()) {
        ideal_path_ = bt::PathPlanner::astar_path(start_.first, start_.second, goal_.first, goal_.second, map_);
    }
    
    job_id_received_ = true;
    
    log("INFO", "Received JobTop for agent " + std::to_string(agent_id_) + 
        ": job_id=" + job_id_ + 
        ", start=(" + std::to_string(start_.first) + "," + std::to_string(start_.second) + ")" +
        ", goal=(" + std::to_string(goal_.first) + "," + std::to_string(goal_.second) + ")" +
        ", leader=" + (is_leader_ ? "YES" : "NO"));
    
    try_initialize();
}

void PathPlanningHandler::handle_zone_pop_message(const std_msgs::msg::Int32::SharedPtr msg) {
    total_agents_ = msg->data;
    zone_pop_received_ = true;
    
    log("INFO", "Received zone_pop (total_agents): " + std::to_string(total_agents_));
    
    try_initialize();
}

void PathPlanningHandler::set_command_publish_callback(CommandPublishCallback callback) {
    command_publish_callback_ = callback;
}

void PathPlanningHandler::set_log_callback(LogCallback callback) {
    log_callback_ = callback;
}

std::pair<int, int> PathPlanningHandler::get_current_position() const {
    if (!initialized_ || !db_) {
        return {-1, -1};
    }
    
    AgentState state = db_->getState(agent_id_);
    return {state.current_x, state.current_y};
}

bool PathPlanningHandler::is_goal_reached() const {
    if (!initialized_ || !db_) {
        return false;
    }
    
    AgentState state = db_->getState(agent_id_);
    return state.goal_reached;
}

void PathPlanningHandler::force_tick_processing() {
    if (initialized_) {
        process_tick(current_tick_);
    }
}

void PathPlanningHandler::reset_state() {
    current_tick_ = 0;
    if (initialized_ && state_updater_) {
        initialize_agent_state();
    }
}

void PathPlanningHandler::try_initialize() {
    if (job_id_received_ && zone_pop_received_ && !initialized_) {
        initialized_ = true;

        // Initialize agent state in database FIRST
        initialize_agent_state();

        // Initialize BigTank shared database
        bt::BigTank::initialize_shared_db(db_);

        // Create StateUpdater AFTER agent state is initialized
        std::string agents_csv_path = config_.agents_csv_path;
        if (agents_csv_path.empty()) {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("bt");
            agents_csv_path = package_share_directory + "/agents.csv";
        }
        
        state_updater_ = std::make_unique<bt::StateUpdater>(node_, agent_id_, db_, total_agents_, agents_csv_path);

        // Set up tick change callback
        state_updater_->set_tick_change_callback([this](int new_tick) {
            this->on_tick_change(new_tick);
        });

        log("INFO", "Agent " + std::to_string(agent_id_) + " initialized with leadership: " + 
            (is_leader_ ? "YES (LEADER)" : "NO"));

        // Start with tick 0 - leader will advance when all agents are ready
        current_tick_ = 0;
        if (state_updater_->get_current_global_tick() == 0) {
            log("INFO", "Agent " + std::to_string(agent_id_) + " starting initial tick processing");
            // DO NOT call process_tick() here - wait for tick_change_callback after state synchronization
            // Instead, just publish initial state to trigger acknowledgment system
            state_updater_->publish_state_for_tick(current_tick_);
        }

        log("INFO", "Agent " + std::to_string(agent_id_) + 
            " start: (" + std::to_string(start_.first) + "," + std::to_string(start_.second) + ")" +
            ", job_id: " + job_id_ + 
            ", goal: (" + std::to_string(goal_.first) + "," + std::to_string(goal_.second) + ")");
        log("INFO", "Loaded map of size " + std::to_string(map_.size()) + "x" + 
            std::to_string(map_.empty() ? 0 : map_[0].size()));
        log("INFO", "Ideal path length: " + std::to_string(ideal_path_.size()));
    }
}

void PathPlanningHandler::initialize_agent_state() {
    AgentState initial_state;
    initial_state.agent_id = agent_id_;
    initial_state.current_x = start_.first;
    initial_state.current_y = start_.second;
    initial_state.priority = priority_;
    initial_state.job_id = job_id_;
    initial_state.goal_x = goal_.first;
    initial_state.goal_y = goal_.second;
    initial_state.is_moving = false;
    initial_state.is_leader = is_leader_;
    initial_state.current_tick = 0;
    initial_state.goal_reached = false;
    initial_state.force = {0.0, 0.0};
    initial_state.chain_force = {0.0, 0.0};
    initial_state.stuck_counter = 0;
    initial_state.force_multiplier = 1.0;
    initial_state.timestamp = std::chrono::system_clock::now();
    
    // Pre-populate next_moves with exactly 2 moves from ideal path
    // This is critical for collision detection to work properly
    initial_state.next_moves.clear();
    if (!ideal_path_.empty() && ideal_path_.size() >= 2) {
        std::pair<int, int> first_move = ideal_path_[1]; // Next immediate move
        std::pair<int, int> second_move = ideal_path_.size() >= 3 ? ideal_path_[2] : first_move;
        
        initial_state.next_moves.push_back(first_move);
        initial_state.next_moves.push_back(second_move);
        
        log("INFO", "Agent " + std::to_string(agent_id_) + 
            " initialized with next_moves: (" + std::to_string(first_move.first) + "," + std::to_string(first_move.second) + ")" +
            ", (" + std::to_string(second_move.first) + "," + std::to_string(second_move.second) + ")");
    } else {
        // No ideal path, stay at current position
        std::pair<int, int> current_pos = {start_.first, start_.second};
        initial_state.next_moves.push_back(current_pos);
        initial_state.next_moves.push_back(current_pos);
        
        log("WARN", "Agent " + std::to_string(agent_id_) + 
            " has no ideal path, staying at current position (" + std::to_string(start_.first) + "," + std::to_string(start_.second) + ")");
    }
    
    db_->setState(agent_id_, initial_state);
    log("INFO", "Agent " + std::to_string(agent_id_) + " state initialized in database");
}

void PathPlanningHandler::on_tick_change(int new_tick) {
    if (new_tick > current_tick_) {
        current_tick_ = new_tick;
        process_tick(current_tick_);
    }
}

void PathPlanningHandler::process_tick(int tick) {
    // Add delay to slow down tick processing for observation
    if (config_.tick_delay_seconds > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(config_.tick_delay_seconds));
    }
    
    AgentState current_state = db_->getState(agent_id_);
    current_state.current_tick = tick;
    current_state.timestamp = std::chrono::system_clock::now();
     
    // Use BigTank algorithm to calculate next move
    AgentState new_state = bt::BigTank::calculate_next_move(current_state, ideal_path_, goal_, map_);
    
    // Check if BigTank provided a valid next move
    if (!new_state.next_moves.empty()) {
        // Execute the move
        std::pair<int, int> next_pos = new_state.next_moves[0];
        
        // Validate the move is within bounds and not blocked
        if (next_pos.first >= 0 && next_pos.first < (int)map_[0].size() &&
            next_pos.second >= 0 && next_pos.second < (int)map_.size() &&
            map_[next_pos.second][next_pos.first] == 0) {
            
            // Update position
            new_state.current_x = next_pos.first;
            new_state.current_y = next_pos.second;
            
            // Set is_moving based on whether the next position is different from current position
            new_state.is_moving = (next_pos.first != current_state.current_x || next_pos.second != current_state.current_y);
            
            // Determine direction and publish command if moving
            if (new_state.is_moving && command_publish_callback_) {
                int direction = 0; // 0 = no movement, 1 = right, 2 = up, 3 = left, 4 = down
                int dx = next_pos.first - current_state.current_x;
                int dy = next_pos.second - current_state.current_y;
                
                if (dx > 0) {
                    direction = 4; // Right
                } else if (dy < 0) {
                    direction = 1; // Up (assuming y decreases going up)
                } else if (dx < 0) {
                    direction = 3; // Left
                } else if (dy > 0) {
                    direction = 2; // Down (assuming y increases going down)
                }
                
                // Publish direction command via callback
                dirac_msgs::msg::AgentCommand command_msg;
                command_msg.direction = direction;
                command_publish_callback_(command_msg);
                
                log("INFO", "Agent " + std::to_string(agent_id_) + " published direction command: " + std::to_string(direction));
            }
            
            // Log movement
            if (new_state.is_moving) {
                log("INFO", "Agent " + std::to_string(agent_id_) + 
                    " moved from (" + std::to_string(current_state.current_x) + "," + std::to_string(current_state.current_y) + ")" +
                    " to (" + std::to_string(new_state.current_x) + "," + std::to_string(new_state.current_y) + ")" +
                    " with force (" + std::to_string(new_state.force.first) + "," + std::to_string(new_state.force.second) + ")" +
                    " at tick " + std::to_string(tick));
            } else {
                log("INFO", "Agent " + std::to_string(agent_id_) + 
                    " staying at (" + std::to_string(new_state.current_x) + "," + std::to_string(new_state.current_y) + ")" +
                    " with force (" + std::to_string(new_state.force.first) + "," + std::to_string(new_state.force.second) + ")" +
                    " at tick " + std::to_string(tick));
            }
        } else {
            // Invalid move, stay in place
            new_state.current_x = current_state.current_x;
            new_state.current_y = current_state.current_y;
            new_state.is_moving = false;
            log("WARN", "Agent " + std::to_string(agent_id_) + 
                " attempted invalid move to (" + std::to_string(next_pos.first) + "," + std::to_string(next_pos.second) + ")" +
                ", staying at (" + std::to_string(current_state.current_x) + "," + std::to_string(current_state.current_y) + ")" +
                " at tick " + std::to_string(tick));
        }
    } else {
        // No move calculated, stay in place
        new_state.current_x = current_state.current_x;
        new_state.current_y = current_state.current_y;
        new_state.is_moving = false;
        log("WARN", "Agent " + std::to_string(agent_id_) + 
            " blocked at (" + std::to_string(current_state.current_x) + "," + std::to_string(current_state.current_y) + ")" +
            ", cannot move at tick " + std::to_string(tick));
    }
    
    // Check if goal reached
    if (new_state.current_x == goal_.first && new_state.current_y == goal_.second) {
        new_state.goal_reached = true;
        if (!current_state.goal_reached) {
            log("INFO", "Agent " + std::to_string(agent_id_) + 
                " REACHED GOAL at (" + std::to_string(new_state.current_x) + "," + std::to_string(new_state.current_y) + ")" +
                " at tick " + std::to_string(tick) + "! Job " + new_state.job_id + " completed.");
        }
    }
    
    // Update state in database
    db_->setState(agent_id_, new_state);
    
    // Debug logging for specific agents if enabled
    if (config_.enable_debug_logging && agent_id_ == 10) {
        log("WARN", "Agent 10 DEBUG: Updated DB with position: (" + 
            std::to_string(new_state.current_x) + "," + std::to_string(new_state.current_y) + ")");
    }
    
    // Trigger distributed coordination by publishing state
    if (state_updater_) {
        state_updater_->publish_state_for_tick(tick);
    }
}

void PathPlanningHandler::log(const std::string& level, const std::string& message) {
    if (log_callback_) {
        log_callback_(level, message);
    } else {
        // Fallback to RCLCPP logging if no callback is set
        if (level == "ERROR") {
            RCLCPP_ERROR(rclcpp::get_logger("PathPlanningHandler"), "%s", message.c_str());
        } else if (level == "WARN") {
            RCLCPP_WARN(rclcpp::get_logger("PathPlanningHandler"), "%s", message.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PathPlanningHandler"), "%s", message.c_str());
        }
    }
}

} // namespace bt
