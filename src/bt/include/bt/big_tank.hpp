#pragma once
#include <vector>
#include <unordered_map>
#include <set>
#include <memory>
#include <string>
#include <cmath>
#include <random>
#include "bt/agent_state.hpp"
#include "bt/agent_state_db.hpp"

namespace bt {

class BigTankAlgorithm {
public:
    // Collision detection result structure
    struct CollisionResult {
        bool has_collision = false;
        AgentState other_agent;
        std::string collision_type = "none";
    };

private:
    // Algorithm constants
    static constexpr int MAX_RECOVERY_THRESHOLD = 10;
    static constexpr int MAX_STUCK_THRESHOLD = 15;
    static constexpr double BASE_FORCE = 1.0;
    static constexpr int CONFLICT_THRESHOLD = 5;
    
    // Core state variables
    bool collision_;
    int stuck_counter_;
    double force_multiplier_;
    int recovery_counter_;
    std::pair<int, int> last_position_;
    std::pair<double, double> chain_force_;
    std::vector<std::pair<int, int>> recovery_;
    std::vector<std::pair<int, int>> registered_path_;
    std::vector<std::pair<int, int>> ideal_path_;
    std::vector<std::vector<int>> map_;
    
    // Conflict tracking
    std::unordered_map<std::string, int> conflict_regions_;
    std::set<std::pair<int, int>> current_conflict_regions_;
    
    // Grid boundaries
    int min_x_, max_x_, min_y_, max_y_;
    
    // References to external systems
    std::shared_ptr<AgentStateDB> agent_state_db_;
    int agent_id_;
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_int_distribution<int> random_dist_;

public:
    BigTankAlgorithm(int agent_id, std::shared_ptr<AgentStateDB> db, 
                     int min_x = 0, int max_x = 100, int min_y = 0, int max_y = 100);
    
    // Set initial paths and map
    void set_ideal_path(const std::vector<std::pair<int, int>>& path);
    void set_map(const std::vector<std::vector<int>>& map);
    
    // Main algorithm entry point
    void execute_tick();
    
    // Core algorithm functions
    void initialize();
    void check_for_deadlock();
    void follow_recovery_path();
    void resolve_conflict(const AgentState& other_agent);
    
    // Movement functions
    void force_based_movement(const std::pair<double, double>& force_vector);
    std::pair<double, double> calculate_force_based_on_conflict(
        const AgentState& self_agent, const AgentState& other_agent);
    std::pair<int, int> select_random_cell(const AgentState& self_agent, 
                                          const AgentState& other_agent);
    std::pair<int, int> select_any_adjacent_cell();
    
    // Path planning
    void recalculate_ideal_path();
    std::pair<double, double> approximate_direction_to_goal(
        const std::pair<int, int>& current, const std::pair<int, int>& goal);
    
    // Collision detection
    bool is_blocked(const std::pair<int, int>& position);
    CollisionResult is_collision(const std::pair<int, int>& intended_position);
    
    // Utility functions
    void reset_chain_force();
    void update_conflict_regions(const std::pair<int, int>& position);
    void clear_old_conflict_regions();
    void broadcast_updated_path(const std::vector<std::pair<int, int>>& path);
    
    // Getters/Setters
    const std::vector<std::pair<int, int>>& get_registered_path() const { return registered_path_; }
    void clear_registered_path() { registered_path_.clear(); }
    bool has_collision() const { return collision_; }
    
private:
    // Helper functions
    std::string position_to_string(const std::pair<int, int>& pos);
    std::pair<int, int> string_to_position(const std::string& pos_str);
    std::vector<std::pair<int, int>> get_adjacent_cells(const std::pair<int, int>& position);
    bool is_within_bounds(const std::pair<int, int>& position);
};

// Compatibility layer for existing agent code
class BigTank {
public:
    static AgentState calculate_next_move(
        const AgentState& current_state,
        const std::vector<std::pair<int, int>>& ideal_path,
        const std::pair<int, int>& goal,
        const std::vector<std::vector<int>>& map
    );
    
private:
    // Map to store per-agent algorithm instances
    static std::unordered_map<int, std::unique_ptr<BigTankAlgorithm>> agent_algorithms_;
    static std::shared_ptr<AgentStateDB> shared_db_;
    
public:
    // Initialize the shared database
    static void initialize_shared_db(std::shared_ptr<AgentStateDB> db);
};

} // namespace bt
