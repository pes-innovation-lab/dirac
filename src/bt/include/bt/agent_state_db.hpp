#pragma once
#include <unordered_map>
#include "bt/agent_state.hpp"

namespace bt {

class AgentStateDB {
public:
    AgentStateDB() = default;
    ~AgentStateDB() = default;

    // Add or update agent state
    void setState(int agent_id, const AgentState& state) {
        db_[agent_id] = state;
    }

    // Update agent state while preserving leadership
    void setStatePreserveLeadership(int agent_id, const AgentState& state) {
        bool was_leader = false;
        auto it = db_.find(agent_id);
        if (it != db_.end()) {
            was_leader = it->second.is_leader;
        }
        
        db_[agent_id] = state;
        db_[agent_id].is_leader = was_leader;
    }

    // Get agent state (returns nullptr if not found)
    const AgentState* getState(int agent_id) const {
        auto it = db_.find(agent_id);
        if (it != db_.end()) {
            return &it->second;
        }
        return nullptr;
    }

    // Get agent state by copy (returns default state if not found)
    AgentState getState(int agent_id) {
        auto it = db_.find(agent_id);
        if (it != db_.end()) {
            return it->second;
        }
        // Return default state if not found
        AgentState default_state{};
        default_state.agent_id = agent_id;
        return default_state;
    }

    // Remove agent state
    void removeState(int agent_id) {
        db_.erase(agent_id);
    }

    // Get all agent states
    const std::unordered_map<int, AgentState>& getAllStates() const {
        return db_;
    }

    // Clear all states
    void clear() {
        db_.clear();
    }

private:
    std::unordered_map<int, AgentState> db_;
};

} // namespace bt
