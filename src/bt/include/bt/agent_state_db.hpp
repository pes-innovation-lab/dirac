#pragma once
#include <unordered_map>
#include "bt/agent_state.hpp"

class AgentStateDB {
public:
    AgentStateDB() = default;
    ~AgentStateDB() = default;

    // Add or update agent state
    void setState(int agent_id, const AgentState& state) {
        db_[agent_id] = state;
    }

    // Get agent state (returns nullptr if not found)
    const AgentState* getState(int agent_id) const {
        auto it = db_.find(agent_id);
        if (it != db_.end()) {
            return &it->second;
        }
        return nullptr;
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
