#include "bt/big_tank.hpp"
#include "bt/path_planner.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <chrono>

namespace bt {

BigTankAlgorithm::BigTankAlgorithm(int agent_id, std::shared_ptr<AgentStateDB> db, 
                                   int min_x, int max_x, int min_y, int max_y) 
    : agent_id_(agent_id)
    , agent_state_db_(db)
    , min_x_(min_x)
    , max_x_(max_x)
    , min_y_(min_y)
    , max_y_(max_y)
    , collision_(false)
    , stuck_counter_(0)
    , force_multiplier_(1.0)
    , recovery_counter_(0)
    , last_position_({-1, -1})
    , chain_force_({0.0, 0.0})
    , rng_(std::chrono::steady_clock::now().time_since_epoch().count())
    , random_dist_(0, 3) {
}

void BigTankAlgorithm::set_ideal_path(const std::vector<std::pair<int, int>>& path) {
    ideal_path_ = path;
}

void BigTankAlgorithm::set_map(const std::vector<std::vector<int>>& map) {
    map_ = map;
}

void BigTankAlgorithm::execute_tick() {
    // Reset collision status for the current tick
    collision_ = false;
    
    // Check if this is the very first run
    if (last_position_.first == -1 && last_position_.second == -1) {
        initialize();
    }
    
    // Check if agent has reached its goal
    AgentState self_state = agent_state_db_->getState(agent_id_);
    if (self_state.current_x == self_state.goal_x && self_state.current_y == self_state.goal_y) {
        // Agent has reached goal, stop processing
        RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                   "Agent %d has reached goal (%d,%d), stopping algorithm execution", 
                   agent_id_, self_state.goal_x, self_state.goal_y);
        
        // Clear next_moves to indicate no further movement
        AgentState updated_state = self_state;
        updated_state.next_moves = self_state.next_moves; // Preserve next_moves for consistency
        updated_state.force = {0.0, 0.0};  // No force needed at goal
        agent_state_db_->setState(agent_id_, updated_state);
        return;
    }
    
    // Check for deadlock and update stuck counter and force multiplier
    check_for_deadlock();
    
    // Clear old conflict regions periodically (every 10 ticks)
    static int conflict_cleanup_counter = 0;
    if (++conflict_cleanup_counter % 10 == 0) {
        clear_old_conflict_regions();
    }
    
    // Main algorithm flow
    if (!collision_ && !recovery_.empty()) {
        // Try to return to the original path using recovery points
        follow_recovery_path();
        
        // Check for excessive recovery to trigger A* re-planning
        if (recovery_counter_ >= MAX_RECOVERY_THRESHOLD) {
            recalculate_ideal_path();
        }
    } else {
        AgentState self_state = agent_state_db_->getState(agent_id_);
        std::pair<int, int> intended_next_pos = {self_state.current_x, self_state.current_y};
        
        // Determine intended next position from ideal path
        if (!ideal_path_.empty()) {
            // Find current position in path and get next step
            for (size_t i = 0; i < ideal_path_.size() - 1; ++i) {
                if (ideal_path_[i].first == self_state.current_x && 
                    ideal_path_[i].second == self_state.current_y) {
                    intended_next_pos = ideal_path_[i + 1];
                    break;
                }
            }
        }
        
        // Check for static obstacles first
        if (is_blocked(intended_next_pos)) {
            RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                       "Agent %d path blocked by static obstacle", agent_id_);
            recalculate_ideal_path();
        } else {
            // Then check for potential collisions if we move to intended position
            CollisionResult collision_result = is_collision(intended_next_pos);
            if (collision_result.has_collision) {
                collision_ = true;
                resolve_conflict(collision_result.other_agent);
            } else if (!collision_) {
                // No recovery needed and no collision detected, proceed with normal movement
                if (ideal_path_.empty()) {
                    RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                               "Agent %d ideal_path is empty. Moving randomly toward goal.", agent_id_);
                    std::pair<double, double> force = approximate_direction_to_goal(
                        {self_state.current_x, self_state.current_y}, 
                        {self_state.goal_x, self_state.goal_y});
                    force_based_movement(force);
                } else {
                    // Calculate force vector toward the intended next position
                    std::pair<double, double> force = {
                        static_cast<double>(intended_next_pos.first - self_state.current_x),
                        static_cast<double>(intended_next_pos.second - self_state.current_y)
                    };
                    force_based_movement(force);
                }
            }
        }
    }
}

void BigTankAlgorithm::initialize() {
    collision_ = false;
    chain_force_ = {0.0, 0.0};
    recovery_.clear();
    registered_path_.clear();
    stuck_counter_ = 0;
    last_position_ = {-1, -1};
    force_multiplier_ = 1.0;
    recovery_counter_ = 0;
    
    // Get current state (should already have next_moves populated by agent initialization)
    AgentState self_state = agent_state_db_->getState(agent_id_);
    
    // Check if next_moves are already populated
    if (!self_state.next_moves.empty()) {
        // Check for collision with the first intended move
        std::pair<int, int> intended_first_move = self_state.next_moves[0];
        CollisionResult collision_result = is_collision(intended_first_move);
        if (collision_result.has_collision) {
            // Set collision flag and use conflict resolution
            collision_ = true;
            resolve_conflict(collision_result.other_agent);
        }
    } else {
        // Fallback: populate next_moves if they weren't set during agent initialization
        RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                   "Agent %d next_moves not populated during agent init, populating now", agent_id_);
        
        AgentState updated_state = self_state;
        updated_state.next_moves.clear();
        
        if (!ideal_path_.empty() && ideal_path_.size() >= 2) {
            std::pair<int, int> intended_first_move = ideal_path_[1]; // Next immediate move
            std::pair<int, int> intended_second_move = ideal_path_.size() >= 3 ? ideal_path_[2] : intended_first_move;
            
            // Check for collision with the intended first move
            CollisionResult collision_result = is_collision(intended_first_move);
            if (collision_result.has_collision) {
                // Set collision flag and use conflict resolution
                collision_ = true;
                resolve_conflict(collision_result.other_agent);
            } else {
                // Validate moves before adding them to next_moves
                std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
                
                if (is_valid_move(current_pos, intended_first_move)) {
                    updated_state.next_moves.push_back(intended_first_move);
                    
                    // Validate second move as well
                    if (is_valid_move(intended_first_move, intended_second_move)) {
                        updated_state.next_moves.push_back(intended_second_move);
                    } else {
                        // If second move is invalid, stay at first move position
                        updated_state.next_moves.push_back(intended_first_move);
                        RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                                   "Agent %d: Second move (%d,%d) invalid, staying at first move (%d,%d)", 
                                   agent_id_, intended_second_move.first, intended_second_move.second,
                                   intended_first_move.first, intended_first_move.second);
                    }
                } else {
                    // First move is invalid, stay in current position
                    updated_state.next_moves.push_back(current_pos);
                    updated_state.next_moves.push_back(current_pos);
                    RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                               "Agent %d: First move (%d,%d) invalid, staying at current position (%d,%d)", 
                               agent_id_, intended_first_move.first, intended_first_move.second,
                               current_pos.first, current_pos.second);
                }
                agent_state_db_->setState(agent_id_, updated_state);
            }
        } else {
            // No ideal path, set current position as both moves (stay put)
            std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
            updated_state.next_moves.push_back(current_pos);
            updated_state.next_moves.push_back(current_pos);
            agent_state_db_->setState(agent_id_, updated_state);
        }
    }
}

void BigTankAlgorithm::check_for_deadlock() {
    AgentState self_state = agent_state_db_->getState(agent_id_);
    std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
    
    if (current_pos == last_position_) {
        // Agent hasn't moved, increment stuck counter
        stuck_counter_++;
        
        // Increase force multiplier based on stuck count and priority
        force_multiplier_ = std::pow(self_state.priority, stuck_counter_);
        
        // Update the force_multiplier in AgentState
        AgentState updated_state = self_state;
        updated_state.force_multiplier = force_multiplier_;
        agent_state_db_->setState(agent_id_, updated_state);
        
        // Log the deadlock
        if (stuck_counter_ % 5 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                       "Agent %d is stuck for %d ticks, force multiplier: %f", 
                       agent_id_, stuck_counter_, force_multiplier_);
        }
        
        // If stuck for too long, take drastic measures
        if (stuck_counter_ > MAX_STUCK_THRESHOLD) {
            std::pair<int, int> next_move = select_any_adjacent_cell();
            
            // Validate the drastic move
            std::pair<int, int> current_pos = {agent_state_db_->getState(agent_id_).current_x, 
                                              agent_state_db_->getState(agent_id_).current_y};
            
            if (is_valid_move(current_pos, next_move)) {
                registered_path_.push_back(next_move);
                
                // Update agent's next_moves for drastic measures - ALWAYS maintain exactly 2 moves
                AgentState drastic_state = agent_state_db_->getState(agent_id_);
                drastic_state.next_moves.clear();
                drastic_state.next_moves.push_back(next_move);
                
                // Add a second random adjacent move if possible, otherwise duplicate first move
                std::pair<int, int> second_move = next_move; // Default
                std::vector<std::pair<int, int>> adjacent = get_adjacent_cells(next_move);
                for (const auto& adj : adjacent) {
                    if (is_valid_move(next_move, adj) && adj != next_move) {
                        second_move = adj;
                        break;
                    }
                }
                
                // ALWAYS add exactly 2 moves
                drastic_state.next_moves.push_back(second_move);
                
                agent_state_db_->setState(agent_id_, drastic_state);
            } else {
                // Even drastic move is invalid, stay in place
                AgentState stay_state = agent_state_db_->getState(agent_id_);
                stay_state.next_moves.clear();
                stay_state.next_moves.push_back(current_pos);
                stay_state.next_moves.push_back(current_pos);
                agent_state_db_->setState(agent_id_, stay_state);
                
                RCLCPP_ERROR(rclcpp::get_logger("BigTank"), 
                            "Agent %d: Even drastic move invalid, staying in place at (%d,%d)", 
                            agent_id_, current_pos.first, current_pos.second);
            }
            
            reset_chain_force();
            recovery_.clear();
            recovery_counter_ = 0;
        }
    } else {
        // Agent has moved, reset stuck counter
        stuck_counter_ = 0;
        force_multiplier_ = 1.0;
        
        // Update the force_multiplier in AgentState
        AgentState updated_state = self_state;
        updated_state.force_multiplier = force_multiplier_;
        agent_state_db_->setState(agent_id_, updated_state);
    }
    
    // Update last position for next tick
    last_position_ = current_pos;
}

void BigTankAlgorithm::force_based_movement(const std::pair<double, double>& force_vector) {
    AgentState self_state = agent_state_db_->getState(agent_id_);
    std::pair<int, int> next_move;
    
    if (std::abs(force_vector.first) > std::abs(force_vector.second)) {
        next_move = {
            self_state.current_x + (force_vector.first > 0 ? 1 : -1), 
            self_state.current_y
        };
    } else {
        next_move = {
            self_state.current_x, 
            self_state.current_y + (force_vector.second > 0 ? 1 : -1)
        };
    }
    
    // Append the calculated next move to the registered path
    registered_path_.push_back(next_move);
    
    // Update agent's next_moves based on force application - ALWAYS maintain exactly 2 moves
    AgentState updated_state = self_state;
    updated_state.next_moves.clear();
    
    // Validate the force-based move
    std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
    if (is_valid_move(current_pos, next_move)) {
        updated_state.next_moves.push_back(next_move); // First move is the calculated move
        
        // Calculate second move based on ideal path or goal direction
        std::pair<int, int> second_move = next_move; // Default to staying at next_move
        if (!ideal_path_.empty()) {
            // Find next_move position in ideal path and get the subsequent move
            for (size_t i = 0; i < ideal_path_.size() - 1; ++i) {
                if (ideal_path_[i] == next_move && i + 1 < ideal_path_.size()) {
                    second_move = ideal_path_[i + 1];
                    break;
                }
            }
        } else {
            // If no ideal path, calculate direction toward goal
            if (next_move.first != self_state.goal_x || next_move.second != self_state.goal_y) {
                if (std::abs(self_state.goal_x - next_move.first) > std::abs(self_state.goal_y - next_move.second)) {
                    second_move = {
                        next_move.first + (self_state.goal_x - next_move.first > 0 ? 1 : -1),
                        next_move.second
                    };
                } else {
                    second_move = {
                        next_move.first,
                        next_move.second + (self_state.goal_y - next_move.second > 0 ? 1 : -1)
                    };
                }
            }
        }
        
        // Validate second move and ensure it's within bounds
        if (!is_valid_move(next_move, second_move)) {
            second_move = next_move; // Stay at next_move if second move is invalid
            RCLCPP_DEBUG(rclcpp::get_logger("BigTank"), 
                        "Agent %d: Second move in force_based_movement invalid, staying at (%d,%d)", 
                        agent_id_, next_move.first, next_move.second);
        }
        
        // ALWAYS add exactly 2 moves
        updated_state.next_moves.push_back(second_move);
    } else {
        // Force-based move is invalid, stay in current position
        updated_state.next_moves.push_back(current_pos);
        updated_state.next_moves.push_back(current_pos);
        RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                   "Agent %d: Force-based move to (%d,%d) invalid, staying at current position (%d,%d)", 
                   agent_id_, next_move.first, next_move.second, current_pos.first, current_pos.second);
    }
    
    updated_state.force = force_vector;  // Store the force that was applied
    agent_state_db_->setState(agent_id_, updated_state);
}

void BigTankAlgorithm::resolve_conflict(const AgentState& other_agent) {
    // Update conflict regions tracking to help future path planning
    AgentState self_state = agent_state_db_->getState(agent_id_);
    std::pair<int, int> next_pos = {self_state.current_x, self_state.current_y};
    if (!self_state.next_moves.empty()) {
        next_pos = self_state.next_moves[0];
    }
    update_conflict_regions(next_pos);
    
    // Check what our intended next move would be without conflict (from ideal path)
    std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
    std::pair<int, int> intended_ideal_move = current_pos; // Default to staying put
    
    // Find our intended next move from the ideal path
    if (!ideal_path_.empty()) {
        for (size_t i = 0; i < ideal_path_.size() - 1; ++i) {
            if (ideal_path_[i] == current_pos) {
                intended_ideal_move = ideal_path_[i + 1];
                break;
            }
        }
    }

    // Apply force from other agent upon itself
    std::pair<double, double> self_force = {0.0, 0.0};
    std::pair<double, double> other_force = {0.0, 0.0};
    
    // Self force: own intended direction weighted by own priority
    if (!self_state.next_moves.empty()) {
        self_force = {
            static_cast<double>(self_state.next_moves[0].first - self_state.current_x) * BASE_FORCE * self_state.priority,
            static_cast<double>(self_state.next_moves[0].second - self_state.current_y) * BASE_FORCE * self_state.priority
        };
    }
    
    // Other agent force: other agent's intended direction weighted by OTHER agent's priority
    if (!other_agent.next_moves.empty()) {
        other_force = {
            static_cast<double>(other_agent.next_moves[0].first - other_agent.current_x) * BASE_FORCE * other_agent.priority,
            static_cast<double>(other_agent.next_moves[0].second - other_agent.current_y) * BASE_FORCE * other_agent.priority
        };
    }
    
    // Combined force applied to self (both forces influence this agent's movement)
    std::pair<double, double> force_on_self = {
        self_force.first + other_force.first,
        self_force.second + other_force.second
    };
    
    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
               "Agent %d conflict resolution - Self force: (%.1f,%.1f), Other force: (%.1f,%.1f), Combined: (%.1f,%.1f), Priority: agent %d %d, agent %d %d", 
               agent_id_, self_force.first, self_force.second, other_force.first, other_force.second, 
               force_on_self.first, force_on_self.second, agent_id_, self_state.priority, other_agent.agent_id, other_agent.priority);

    // Calculate where the force will take us
    std::pair<int, int> force_destination;
    if (std::abs(force_on_self.first) > std::abs(force_on_self.second)) {
        force_destination = {
            self_state.current_x + (force_on_self.first > 0 ? 1 : -1), 
            self_state.current_y
        };
    } else {
        force_destination = {
            self_state.current_x, 
            self_state.current_y + (force_on_self.second > 0 ? 1 : -1)
        };
    }
    
    // Only add to recovery if the force will take us away from our intended ideal path move
    if (intended_ideal_move != current_pos && force_destination != intended_ideal_move) {
        recovery_.push_back(current_pos);
        RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                   "Agent %d adding recovery point (%d,%d) - force pushes to (%d,%d) instead of ideal (%d,%d)", 
                   agent_id_, current_pos.first, current_pos.second, 
                   force_destination.first, force_destination.second,
                   intended_ideal_move.first, intended_ideal_move.second);
    }

    // Move based on combined force directly
    force_based_movement(force_on_self);
}

void BigTankAlgorithm::follow_recovery_path() {
    AgentState self_state = agent_state_db_->getState(agent_id_);
    
    // Return to original path by following recovery points in reverse order
    std::pair<int, int> current_pos = {self_state.current_x, self_state.current_y};
    if (!recovery_.empty() && current_pos != recovery_.back()) {
        std::pair<int, int> intended_next_move = recovery_.back();
        
        // First check if the intended move is blocked by obstacles
        if (is_blocked(intended_next_move)) {
            RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                       "Agent %d recovery path blocked by static obstacle", agent_id_);
            recalculate_ideal_path();
            return;
        }
        
        // Then check for potential collisions with other agents
        CollisionResult collision_result = is_collision(intended_next_move);
        if (collision_result.has_collision) {
            collision_ = true;
            resolve_conflict(collision_result.other_agent);
            return;
        } else {
            // No conflict, check if the recovery move is valid
            if (is_valid_move(current_pos, intended_next_move)) {
                collision_ = false;
                
                recovery_.pop_back();
                registered_path_.push_back(intended_next_move);
                recovery_counter_++;
                
                // Update agent's next_moves for recovery - ALWAYS maintain exactly 2 moves
                AgentState updated_state = self_state;
                updated_state.next_moves.clear();
                updated_state.next_moves.push_back(intended_next_move);
                
                // Calculate second move from remaining recovery path or ideal path
                std::pair<int, int> second_move = intended_next_move; // Default
                if (!recovery_.empty()) {
                    second_move = recovery_.back();
                } else if (!ideal_path_.empty()) {
                    // Find current position in ideal path and add next step
                    for (size_t i = 0; i < ideal_path_.size() - 1; ++i) {
                        if (ideal_path_[i] == intended_next_move && i + 1 < ideal_path_.size()) {
                            second_move = ideal_path_[i + 1];
                            break;
                        }
                    }
                }
                
                // Validate second move
                if (!is_valid_move(intended_next_move, second_move)) {
                    second_move = intended_next_move; // Stay at first move if second is invalid
                    RCLCPP_DEBUG(rclcpp::get_logger("BigTank"), 
                                "Agent %d: Second recovery move invalid, staying at (%d,%d)", 
                                agent_id_, intended_next_move.first, intended_next_move.second);
                }
                
                // ALWAYS add exactly 2 moves
                updated_state.next_moves.push_back(second_move);
                
                agent_state_db_->setState(agent_id_, updated_state);
            } else {
                // Recovery move is invalid, recalculate path
                RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                           "Agent %d recovery move to (%d,%d) invalid, recalculating path", 
                           agent_id_, intended_next_move.first, intended_next_move.second);
                recalculate_ideal_path();
                return;
            }
        }
    }
    
    // Once we've returned to the original path, clear recovery
    if (recovery_.empty() || current_pos == recovery_.back()) {
        recovery_.clear();
        recovery_counter_ = 0;
        stuck_counter_ = 0;
        force_multiplier_ = 1.0;
    }
}

bool BigTankAlgorithm::is_blocked(const std::pair<int, int>& position) {
    // Check if position is outside grid boundaries
    if (!is_within_bounds(position)) {
        return true;
    }
    
    // Check for static obstacles in the map
    // Map is accessed as map[y][x], position is (x, y)
    if (!map_.empty() && position.second < (int)map_.size() && 
        position.first < (int)map_[position.second].size()) {
        if (map_[position.second][position.first] == 1) {
            return true; // Blocked by static obstacle
        }
    }
    
    return false;
}

bool BigTankAlgorithm::is_valid_move(const std::pair<int, int>& from, const std::pair<int, int>& to) {
    // Check if destination is blocked (out of bounds or obstacle)
    if (is_blocked(to)) {
        RCLCPP_DEBUG(rclcpp::get_logger("BigTank"), 
                    "Agent %d move from (%d,%d) to (%d,%d) invalid: destination blocked", 
                    agent_id_, from.first, from.second, to.first, to.second);
        return false;
    }
    
    // Check if move is within allowed Manhattan distance (typically 1 for grid movement)
    if (!is_move_within_manhattan_distance(from, to, 1)) {
        RCLCPP_DEBUG(rclcpp::get_logger("BigTank"), 
                    "Agent %d move from (%d,%d) to (%d,%d) invalid: exceeds Manhattan distance", 
                    agent_id_, from.first, from.second, to.first, to.second);
        return false;
    }
    
    return true;
}

bool BigTankAlgorithm::is_move_within_manhattan_distance(const std::pair<int, int>& from, const std::pair<int, int>& to, int max_distance) {
    int manhattan_distance = std::abs(to.first - from.first) + std::abs(to.second - from.second);
    return manhattan_distance <= max_distance;
}

BigTankAlgorithm::CollisionResult BigTankAlgorithm::is_collision(const std::pair<int, int>& intended_position) {
    CollisionResult result;
    AgentState self_state = agent_state_db_->getState(agent_id_);
    std::pair<int, int> self_current = {self_state.current_x, self_state.current_y};
    
    // Get all agent states
    auto all_states = agent_state_db_->getAllStates();
    
    for (const auto& [other_agent_id, other_state] : all_states) {
        if (other_agent_id != agent_id_) {
            std::pair<int, int> other_current = {other_state.current_x, other_state.current_y};
            
            // Check collisions with ALL planned moves from other agent
            for (size_t i = 0; i < other_state.next_moves.size(); ++i) {
                const auto& other_intended = other_state.next_moves[i];
                
                // Vertex collision - both agents plan to occupy the same position
                if (other_intended == intended_position) {
                    result.has_collision = true;
                    result.other_agent = other_state;
                    result.collision_type = "vertex";
                    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                               "Agent %d: VERTEX COLLISION with agent %d at (%d,%d)", 
                               agent_id_, other_agent_id, intended_position.first, intended_position.second);
                    return result;
                }
                
                // Swap collision - agents exchange positions
                if (other_current == intended_position && other_intended == self_current) {
                    result.has_collision = true;
                    result.other_agent = other_state;
                    result.collision_type = "swap";
                    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                               "Agent %d: SWAP COLLISION with agent %d", 
                               agent_id_, other_agent_id);
                    return result;
                }
                
                // Edge collision - agents cross the same edge in opposite directions
                if (self_current == other_intended && intended_position == other_current) {
                    result.has_collision = true;
                    result.other_agent = other_state;
                    result.collision_type = "edge";
                    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                               "Agent %d: EDGE COLLISION with agent %d", 
                               agent_id_, other_agent_id);
                    return result;
                }
            }
        }
    }
    
    return result; // No collision detected
}

std::pair<int, int> BigTankAlgorithm::select_any_adjacent_cell() {
    AgentState self_state = agent_state_db_->getState(agent_id_);
    std::vector<std::pair<int, int>> possible_moves = get_adjacent_cells({self_state.current_x, self_state.current_y});
    
    // Filter by grid boundaries
    auto it = std::remove_if(possible_moves.begin(), possible_moves.end(),
        [this](const std::pair<int, int>& pos) { return !is_within_bounds(pos); });
    possible_moves.erase(it, possible_moves.end());
    
    if (possible_moves.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("BigTank"), 
                    "Agent %d is completely trapped and cannot select any adjacent cell.", agent_id_);
        return {self_state.current_x, self_state.current_y}; // Stay put
    }
    
    // Return a random move
    int random_index = random_dist_(rng_) % possible_moves.size();
    return possible_moves[random_index];
}

void BigTankAlgorithm::reset_chain_force() {
    chain_force_ = {0.0, 0.0};
}

std::pair<double, double> BigTankAlgorithm::approximate_direction_to_goal(
    const std::pair<int, int>& current, const std::pair<int, int>& goal) {
    double dx = goal.first - current.first;
    double dy = goal.second - current.second;
    
    // Normalize to prevent excessive force
    double magnitude = std::sqrt(dx*dx + dy*dy);
    if (magnitude > 0) {
        dx = dx / magnitude;
        dy = dy / magnitude;
    }
    
    return {dx, dy};
}

void BigTankAlgorithm::recalculate_ideal_path() {
    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
               "Agent %d recalculating ideal path.", agent_id_);
    
    AgentState self_state = agent_state_db_->getState(agent_id_);
    
    if (!map_.empty()) {
        // Use A* pathfinding to recalculate path
        ideal_path_ = bt::PathPlanner::astar_path(
            self_state.current_x, self_state.current_y,
            self_state.goal_x, self_state.goal_y,
            map_
        );
        
        RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                   "Agent %d recalculated path with %zu steps", 
                   agent_id_, ideal_path_.size());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("BigTank"), 
                   "Agent %d cannot recalculate path - no map available", agent_id_);
    }
    
    // Reset local state related to recovery and being stuck
    recovery_.clear();
    recovery_counter_ = 0;
    stuck_counter_ = 0;
    force_multiplier_ = 1.0;
    chain_force_ = {0.0, 0.0};
    
    // Update the force_multiplier in AgentState
    AgentState updated_state = agent_state_db_->getState(agent_id_);
    updated_state.force_multiplier = force_multiplier_;
    agent_state_db_->setState(agent_id_, updated_state);
    
    // TODO: Broadcast the recalculated path to other agents
    broadcast_updated_path({}); // Empty path for now
}

void BigTankAlgorithm::broadcast_updated_path(const std::vector<std::pair<int, int>>& path) {
    RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
               "Agent %d broadcasting updated path", agent_id_);
    
    // TODO: Implement actual broadcasting mechanism
    // This would use ROS2 topics or shared memory to notify other agents
}

void BigTankAlgorithm::update_conflict_regions(const std::pair<int, int>& position) {
    std::string pos_str = position_to_string(position);
    
    if (conflict_regions_.find(pos_str) != conflict_regions_.end()) {
        conflict_regions_[pos_str]++;
    } else {
        conflict_regions_[pos_str] = 1;
    }
    
    // If a region has too many conflicts, mark it for avoidance in future planning
    if (conflict_regions_[pos_str] > CONFLICT_THRESHOLD) {
        current_conflict_regions_.insert(position);
    }
}

void BigTankAlgorithm::clear_old_conflict_regions() {
    for (auto it = conflict_regions_.begin(); it != conflict_regions_.end();) {
        it->second--;
        if (it->second <= 0) {
            std::pair<int, int> pos = string_to_position(it->first);
            current_conflict_regions_.erase(pos);
            it = conflict_regions_.erase(it);
        } else {
            ++it;
        }
    }
}

// Helper functions
std::string BigTankAlgorithm::position_to_string(const std::pair<int, int>& pos) {
    return std::to_string(pos.first) + "," + std::to_string(pos.second);
}

std::pair<int, int> BigTankAlgorithm::string_to_position(const std::string& pos_str) {
    size_t comma_pos = pos_str.find(',');
    int x = std::stoi(pos_str.substr(0, comma_pos));
    int y = std::stoi(pos_str.substr(comma_pos + 1));
    return {x, y};
}

std::vector<std::pair<int, int>> BigTankAlgorithm::get_adjacent_cells(const std::pair<int, int>& position) {
    return {
        {position.first + 1, position.second},
        {position.first - 1, position.second},
        {position.first, position.second + 1},
        {position.first, position.second - 1}
    };
}

bool BigTankAlgorithm::is_within_bounds(const std::pair<int, int>& position) {
    return position.first >= min_x_ && position.first <= max_x_ &&
           position.second >= min_y_ && position.second <= max_y_;
}

// Static compatibility layer implementation
std::unordered_map<int, std::unique_ptr<BigTankAlgorithm>> BigTank::agent_algorithms_;
std::shared_ptr<AgentStateDB> BigTank::shared_db_;

void BigTank::initialize_shared_db(std::shared_ptr<AgentStateDB> db) {
    shared_db_ = db;
}

AgentState BigTank::calculate_next_move(
    const AgentState& current_state,
    const std::vector<std::pair<int, int>>& ideal_path,
    const std::pair<int, int>& goal,
    const std::vector<std::vector<int>>& map
) {
    int agent_id = current_state.agent_id;
    
    // Create algorithm instance for this agent if it doesn't exist
    if (agent_algorithms_.find(agent_id) == agent_algorithms_.end()) {
        if (!shared_db_) {
            RCLCPP_ERROR(rclcpp::get_logger("BigTank"), 
                        "Shared database not initialized! Call BigTank::initialize_shared_db() first.");
            return current_state;
        }
        
        // Create new algorithm instance with map bounds
        int max_x = map.empty() ? 100 : static_cast<int>(map[0].size()) - 1;
        int max_y = map.empty() ? 100 : static_cast<int>(map.size()) - 1;
        
        agent_algorithms_[agent_id] = std::make_unique<BigTankAlgorithm>(
            agent_id, shared_db_, 0, max_x, 0, max_y);
        
        // Set the ideal path and map
        agent_algorithms_[agent_id]->set_ideal_path(ideal_path);
        agent_algorithms_[agent_id]->set_map(map);
        
        RCLCPP_INFO(rclcpp::get_logger("BigTank"), 
                   "Created BigTankAlgorithm instance for agent %d with bounds (0,0) to (%d,%d)", 
                   agent_id, max_x, max_y);
    }
    
    // Execute one tick of the algorithm
    agent_algorithms_[agent_id]->execute_tick();
    
    // Get the updated state from the database
    AgentState updated_state = shared_db_->getState(agent_id);
    
    return updated_state;
}

} // namespace bt
