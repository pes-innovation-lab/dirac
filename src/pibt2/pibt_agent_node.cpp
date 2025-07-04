#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <sys/file.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <algorithm>
#include <optional>
#include <map>
#include <queue>
#include <unordered_set>

// --- Minimal grid-based Graph and Node for path planning ---
struct Position {
    int x, y;
    bool operator==(const Position& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Position& other) const { return !(*this == other); }
};
namespace std {
    template<>
    struct hash<Position> {
        std::size_t operator()(const Position& p) const {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
        }
    };
}

struct GridGraph {
    int width, height;
    std::unordered_set<Position> obstacles;
    GridGraph(int w, int h) : width(w), height(h) {}
    bool inBounds(const Position& p) const {
        return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
    }
    bool passable(const Position& p) const {
        return obstacles.find(p) == obstacles.end();
    }
    std::vector<Position> neighbors(const Position& p) const {
        std::vector<Position> nbrs;
        for (auto [dx, dy] : std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}) {
            Position np{p.x+dx, p.y+dy};
            if (inBounds(np) && passable(np)) nbrs.push_back(np);
        }
        return nbrs;
    }
};

// --- End Graph/Node ---

struct AgentWin {
    int id;
    std::vector<Position> PI; // Ideal window
    std::vector<Position> R;  // Real window (optional, can be used for actual path)
    int l = 0;                // Last secured timestep
    float priority = 0.0;
    int window = 1;
    std::string job_id;
    std::string status;
    Position goal; // Add goal for path planning
};

using ProvisionalPaths = std::map<int, std::vector<Position>>; // agent_id -> PI
using AgentSet = std::set<int>; // agent ids

class PIBTAgentNode : public rclcpp::Node {
public:
    PIBTAgentNode() : Node("pibt_agent_node") {
        const char* env_path = std::getenv("AGENT_JSON_PATH");
        json_file_path_ = env_path ? env_path : "/data/agent_state.json";
        agent_id_ = std::getenv("AGENT_ID") ? std::atoi(std::getenv("AGENT_ID")) : 1;

        // For demo: 10x10 grid, no obstacles
        graph_ = std::make_shared<GridGraph>(10, 10);

        // Initialize JSON if needed
        initialize_json_if_needed();

        json_update_pub_ = this->create_publisher<std_msgs::msg::String>("/agent_json_updates", 10);
        job_status_pub_ = this->create_publisher<std_msgs::msg::String>("/job_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PIBTAgentNode::timestep_callback, this)
        );
    }

private:
    std::string json_file_path_;
    int agent_id_;
    std::shared_ptr<GridGraph> graph_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr job_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int timestep_ = 0;

    // --- Path planning: BFS for shortest path ---
    std::vector<Position> bfs_path(const Position& start, const Position& goal, int max_len) {
        std::queue<std::vector<Position>> q;
        std::unordered_set<Position> visited;
        q.push({start});
        visited.insert(start);
        while (!q.empty()) {
            auto path = q.front(); q.pop();
            Position last = path.back();
            if (last == goal || (int)path.size() >= max_len) return path;
            for (const auto& nbr : graph_->neighbors(last)) {
                if (visited.find(nbr) == visited.end()) {
                    auto new_path = path;
                    new_path.push_back(nbr);
                    q.push(new_path);
                    visited.insert(nbr);
                }
            }
        }
        return {start}; // fallback: stay in place
    }

    // Helper: Initialize JSON file if missing or empty (nlohmann/json version)
    void initialize_json_if_needed() {
        std::ifstream infile(json_file_path_);
        bool needs_init = !infile.good() || infile.peek() == std::ifstream::traits_type::eof();
        infile.close();
        if (needs_init) {
            nlohmann::json doc;
            // Example: 10 agents, each with a unique id and default values
            for (int i = 1; i <= 10; ++i) {
                nlohmann::json a;
                a["id"] = i;
                a["l"] = 0;
                a["priority"] = 1.0 / (1 + i);
                a["window"] = 3;
                a["job_id"] = "job_" + std::to_string(i);
                a["status"] = "moving";
                // PI: start at (i,0)
                a["PI"] = {{{"x", i}, {"y", 0}}};
                // R: empty
                a["R"] = nlohmann::json::array();
                // goal: (i,9)
                a["goal"] = {{"x", i}, {"y", 9}};
                doc["agents"].push_back(a);
            }
            // Write to file
            std::ofstream o(json_file_path_);
            o << doc.dump(4) << std::endl;
        }
    }

    // Helper: Parse agent list from JSON (nlohmann/json version)
    std::vector<AgentWin> parse_agents(const nlohmann::json& doc) {
        std::vector<AgentWin> agents;
        for (const auto& a : doc["agents"]) {
            AgentWin ag;
            ag.id = a["id"];
            ag.l = a.value("l", 0);
            ag.priority = a.value("priority", 0.0);
            ag.window = a.value("window", 1);
            ag.job_id = a.value("job_id", "");
            ag.status = a.value("status", "");
            // Parse PI
            for (const auto& p : a["PI"]) {
                ag.PI.push_back({p["x"], p["y"]});
            }
            // Parse R (optional)
            if (a.contains("R")) {
                for (const auto& p : a["R"]) {
                    ag.R.push_back({p["x"], p["y"]});
                }
            }
            // Parse goal
            if (a.contains("goal")) {
                ag.goal.x = a["goal"]["x"];
                ag.goal.y = a["goal"]["y"];
            } else {
                ag.goal = {0, 0};
            }
            agents.push_back(ag);
        }
        return agents;
    }

    // Helper: Write agents back to JSON (nlohmann/json version)
    void write_agents(nlohmann::json& doc, const std::vector<AgentWin>& agents) {
        doc["agents"] = nlohmann::json::array();
        for (const auto& ag : agents) {
            nlohmann::json a;
            a["id"] = ag.id;
            a["l"] = ag.l;
            a["priority"] = ag.priority;
            a["window"] = ag.window;
            a["job_id"] = ag.job_id;
            a["status"] = ag.status;
            // PI
            a["PI"] = nlohmann::json::array();
            for (const auto& p : ag.PI) {
                a["PI"].push_back({{"x", p.x}, {"y", p.y}});
            }
            // R (optional)
            a["R"] = nlohmann::json::array();
            for (const auto& p : ag.R) {
                a["R"].push_back({{"x", p.x}, {"y", p.y}});
            }
            // goal
            a["goal"] = {{"x", ag.goal.x}, {"y", ag.goal.y}};
            doc["agents"].push_back(a);
        }
    }

    // Helper: Find max timestep registered in PI
    int maxTimestepRegisteredIn(const ProvisionalPaths& PI) {
        int max_t = 0;
        for (const auto& [id, path] : PI) {
            if (!path.empty()) max_t = std::max(max_t, (int)path.size() - 1);
        }
        return max_t;
    }

    // Helper: Register path in PI (now uses BFS path planner)
    void registerPath(AgentWin& ai, int alpha, int beta, ProvisionalPaths& PI) {
        // Plan a path from current position to goal, up to window length
        Position start = (PI[ai.id].empty() ? Position{0,0} : PI[ai.id][ai.l]);
        std::vector<Position> planned = bfs_path(start, ai.goal, beta - ai.l);
        auto& path = PI[ai.id];
        // Ensure path is long enough
        while ((int)path.size() <= beta) path.push_back(path.empty() ? start : path.back());
        // Fill in planned positions for [ai.l+1, beta]
        for (int t = ai.l+1, idx = 1; t <= beta; ++t, ++idx) {
            if (idx < (int)planned.size())
                path[t] = planned[idx];
            else
                path[t] = planned.back(); // stay at goal
        }
        ai.l = beta;
    }

    // Helper: Revoke nodes in PI
    void revokeNodes(AgentWin& ai, int from, int to, ProvisionalPaths& PI) {
        auto& path = PI[ai.id];
        for (int t = from; t <= to && t < (int)path.size(); ++t) {
            // TODO: Set to invalid or previous value
            path[t] = path[from-1];
        }
        ai.l = from-1;
    }

    // Helper: Agents with l < li and vj(lj) == v
    std::vector<int> agentsWithLBelow(int li, const Position& v, const ProvisionalPaths& PI, const std::vector<AgentWin>& agents) {
        std::vector<int> result;
        for (const auto& ag : agents) {
            if (ag.l < li && !PI.at(ag.id).empty() && PI.at(ag.id)[ag.l] == v) {
                result.push_back(ag.id);
            }
        }
        return result;
    }
    // Helper: Agents with l == li and vj(lj) == v, not in R
    std::vector<int> agentsWithLEqual(int li, const Position& v, const ProvisionalPaths& PI, const std::vector<AgentWin>& agents, const AgentSet& R) {
        std::vector<int> result;
        for (const auto& ag : agents) {
            if (ag.l == li && !PI.at(ag.id).empty() && PI.at(ag.id)[ag.l] == v && R.find(ag.id) == R.end()) {
                result.push_back(ag.id);
            }
        }
        return result;
    }

    // winPIBT function
    enum class WinPIBTResult { VALID, INVALID };
    WinPIBTResult winPIBT(AgentWin& ai, int alpha, ProvisionalPaths& PI, AgentSet R, std::vector<AgentWin>& agents) {
        if (ai.l >= alpha) return WinPIBTResult::VALID;
        int beta = std::max(alpha, maxTimestepRegisteredIn(PI));
        if (!validPath(ai, beta, PI)) {
            copeStuck(ai, alpha, PI);
            return WinPIBTResult::INVALID;
        }
        registerPath(ai, alpha, beta, PI);
        R.insert(ai.id);
        while (ai.l < alpha) {
            Position v = PI[ai.id][ai.l + 1]; // target node
            // Resolve conflicts with lower-l agents
            for (int aj_id : agentsWithLBelow(ai.l, v, PI, agents)) {
                auto& aj = *std::find_if(agents.begin(), agents.end(), [&](const AgentWin& a){return a.id==aj_id;});
                winPIBT(aj, aj.l + 1, PI, R, agents);
            }
            // Resolve conflicts with same-l agents not in R
            for (int aj_id : agentsWithLEqual(ai.l, v, PI, agents, R)) {
                auto& aj = *std::find_if(agents.begin(), agents.end(), [&](const AgentWin& a){return a.id==aj_id;});
                if (winPIBT(aj, aj.l + 1, PI, R, agents) == WinPIBTResult::INVALID) {
                    revokeNodes(ai, ai.l + 1, alpha, PI);
                    if (!validPath(ai, beta, PI)) {
                        copeStuck(ai, alpha, PI);
                        return WinPIBTResult::INVALID;
                    } else {
                        registerPath(ai, alpha, beta, PI);
                        continue;
                    }
                }
            }
            // Secure node
            PI[ai.id][ai.l + 1] = v;
            ai.l++;
        }
        return WinPIBTResult::VALID;
    }

    // Helper: Check if path is valid (no conflicts in PI)
    bool validPath(const AgentWin& ai, int beta, const ProvisionalPaths& PI) {
        for (int t = 0; t <= beta; ++t) {
            if (PI.at(ai.id).size() <= t) continue;
            Position my_pos = PI.at(ai.id)[t];
            for (const auto& [other_id, other_path] : PI) {
                if (other_id == ai.id || (int)other_path.size() <= t) continue;
                // Vertex conflict
                if (my_pos == other_path[t]) return false;
                // Edge conflict
                if (t > 0 && my_pos == other_path[t-1] && PI.at(ai.id)[t-1] == other_path[t]) return false;
            }
        }
        return true;
    }

    // Helper: Cope with stuck agent (wait in place)
    void copeStuck(AgentWin& ai, int alpha, ProvisionalPaths& PI) {
        auto& path = PI[ai.id];
        Position wait_pos = (path.empty() ? Position{0,0} : path.back());
        while ((int)path.size() <= alpha) path.push_back(wait_pos);
        for (int t = ai.l+1; t <= alpha; ++t) {
            path[t] = wait_pos;
        }
        ai.l = alpha;
    }

    // Main loop (nlohmann/json version)
    void timestep_callback() {
        // Acquire writer lock for update
        FILE* fp = fopen(json_file_path_.c_str(), "r+");
        if (!fp) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_file_path_.c_str());
            return;
        }
        int fd = fileno(fp);
        if (flock(fd, LOCK_EX) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire file lock");
            fclose(fp);
            return;
        }
        // Read current JSON
        std::ifstream in(json_file_path_);
        nlohmann::json doc;
        in >> doc;
        in.close();
        // Parse agents
        std::vector<AgentWin> agents = parse_agents(doc);
        // Build PI from agents
        ProvisionalPaths PI;
        for (const auto& ag : agents) PI[ag.id] = ag.PI;
        // Update priorities and window sizes (TODO: domain-specific)
        for (auto& ag : agents) {
            ag.priority = 1.0 / (1 + ag.id); // Example: lower id = higher priority
            ag.window = 3; // Example: fixed window size
        }
        // Sort agents by priority (descending)
        std::vector<AgentWin*> sorted_agents;
        for (auto& ag : agents) sorted_agents.push_back(&ag);
        std::sort(sorted_agents.begin(), sorted_agents.end(), [](AgentWin* a, AgentWin* b){ return a->priority > b->priority; });
        int kappa = 0;
        for (size_t j = 0; j < sorted_agents.size(); ++j) {
            AgentWin& ai = *sorted_agents[j];
            if (ai.l <= timestep_) {
                if (j == 0) {
                    winPIBT(ai, timestep_ + ai.window, PI, {}, agents);
                } else {
                    winPIBT(ai, std::min(timestep_ + ai.window, kappa), PI, {}, agents);
                }
            }
            kappa = std::min(kappa, ai.l);
            if (j == 0) kappa = ai.l;
        }
        // Write back PI to agents
        for (auto& ag : agents) ag.PI = PI[ag.id];
        // Job completion detection and status update
        auto it = std::find_if(agents.begin(), agents.end(), [&](const AgentWin& a){return a.id==agent_id_;});
        if (it != agents.end()) {
            AgentWin& my_agent = *it;
            bool completed = false;
            if (!my_agent.PI.empty() && my_agent.PI.back() == my_agent.goal) {
                completed = true;
            }
            if (completed && my_agent.status != "completed") {
                my_agent.status = "completed";
                // Publish job completion as a string
                std_msgs::msg::String job_status_msg;
                job_status_msg.data = "Agent " + std::to_string(agent_id_) + " completed job " + my_agent.job_id;
                job_status_pub_->publish(job_status_msg);
            } else if (!completed) {
                my_agent.status = "moving";
            }
        }
        // Write agents back to JSON
        write_agents(doc, agents);
        // Write back the updated JSON
        std::ofstream out(json_file_path_);
        out << doc.dump(4) << std::endl;
        out.close();
        // Publish the change as a string
        std_msgs::msg::String update_msg;
        update_msg.data = doc.dump();
        json_update_pub_->publish(update_msg);
        // Release lock and close
        flock(fd, LOCK_UN);
        fclose(fp);
        timestep_++;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIBTAgentNode>());
    rclcpp::shutdown();
    return 0;
} 