#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "talk/msg/agent_state.hpp"

#include <string>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <vector>
#include <algorithm>

struct Node {
    int x, y;
    int g, h;
    Node* parent;
    int f() const { return g + h; }
    bool operator>(const Node& other) const { return f() > other.f(); }
};

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

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

std::vector<std::pair<int, int>> astar(int start_x, int start_y, int goal_x, int goal_y) {
    std::vector<std::pair<int, int>> path;
    bool closed[20][20] = {false};
    auto cmp = [](Node* a, Node* b) { return a->f() > b->f(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

    Node* start = new Node{start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y), nullptr};
    open.push(start);

    while (!open.empty()) {
        Node* curr = open.top();
        open.pop();

        if (curr->x == goal_x && curr->y == goal_y) {
            Node* n = curr;
            while (n) {
                path.push_back({n->x, n->y});
                n = n->parent;
            }
            std::reverse(path.begin(), path.end());
            while (!open.empty()) { delete open.top(); open.pop(); }
            return path;
        }

        closed[curr->x][curr->y] = true;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int dir = 0; dir < 4; ++dir) {
            int nx = curr->x + dx[dir];
            int ny = curr->y + dy[dir];
            if (nx < 0 || nx >= 20 || ny < 0 || ny >= 20) continue;
            if (map_[nx][ny] == 1) continue;
            if (closed[nx][ny]) continue;
            int g_new = curr->g + 1;
            int h_new = heuristic(nx, ny, goal_x, goal_y);
            Node* neighbor = new Node{nx, ny, g_new, h_new, curr};
            open.push(neighbor);
        }
    }
    return {};
}

class AgentNode : public rclcpp::Node {
public:
    AgentNode() : Node("agent_node") {
        // declare & get parameters
        declare_parameter("agent_id", "agent1");
        declare_parameter("agent_x", 0.0);
        declare_parameter("agent_y", 0.0);            
        declare_parameter("priority", 0);
        declare_parameter("job_id", "job1");
        declare_parameter("job_x", 0.0);
        declare_parameter("job_y", 0.0);

        agent_id_ = get_parameter("agent_id").as_string();
        agent_x_ = get_parameter("agent_x").as_double();
        agent_y_ = get_parameter("agent_y").as_double();
        priority_ = get_parameter("priority").as_int();
        job_id_ = get_parameter("job_id").as_string();
        job_x_ = get_parameter("job_x").as_double();
        job_y_ = get_parameter("job_y").as_double();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&AgentNode::publish_agent_state, this)
        );

        agent_state_pub_ = this->create_publisher<talk::msg::AgentState>("agent_state", 10);
        agent_state_sub_ = this->create_subscription<talk::msg::AgentState>(
            "agent_state", 10,
            std::bind(&AgentNode::agent_state_callback, this, std::placeholders::_1));
    }

private:
    void agent_state_callback(const talk::msg::AgentState::SharedPtr msg) {
        agent_state_table_[msg->agent_id] = *msg;
        RCLCPP_INFO(this->get_logger(), "Received state from %s", msg->agent_id.c_str());
    }

    void publish_agent_state() {
        int sx = static_cast<int>(agent_x_);
        int sy = static_cast<int>(agent_y_);
        int gx = static_cast<int>(job_x_);
        int gy = static_cast<int>(job_y_);
        auto path = astar(sx, sy, gx, gy);

        std::vector<std::pair<int, int>> next_steps;
        for (size_t i = 0; i < std::min<size_t>(4, path.size()); ++i)
            next_steps.push_back(path[i]);
        while (next_steps.size() < 4 && !next_steps.empty())
            next_steps.push_back(next_steps.back());

        bool blocked = false;
        if (next_steps.size() > 1) {
            int nx = next_steps[1].first;
            int ny = next_steps[1].second;
            for (const auto& kv : agent_state_table_) {
                const auto& other = kv.second;
                if (other.agent_id == agent_id_) continue;
                if (other.priority >= priority_) {
                    if ((static_cast<int>(other.x) == nx && static_cast<int>(other.y) == ny) ||
                        (other.next_steps.size() > 1 &&
                         static_cast<int>(other.next_steps[1].x) == nx &&
                         static_cast<int>(other.next_steps[1].y) == ny)) {
                        blocked = true;
                        break;
                    }
                }
            }
        }

        if (!blocked && next_steps.size() > 1) {
            agent_x_ = next_steps[1].first;
            agent_y_ = next_steps[1].second;
        }

        talk::msg::AgentState msg;
        msg.agent_id = agent_id_;
        msg.x = agent_x_;
        msg.y = agent_y_;
        msg.priority = priority_;
        msg.job_id = job_id_;
        msg.job_x = job_x_;
        msg.job_y = job_y_;
        msg.next_steps.clear();
        for (const auto& step : next_steps) {
            geometry_msgs::msg::Point pt;
            pt.x = step.first;
            pt.y = step.second;
            pt.z = 0.0;
            msg.next_steps.push_back(pt);
        }
        agent_state_pub_->publish(msg);
    }

    std::string agent_id_;
    double agent_x_;
    double agent_y_;
    int priority_;
    std::string job_id_;
    double job_x_;
    double job_y_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<talk::msg::AgentState>::SharedPtr agent_state_pub_;
    rclcpp::Subscription<talk::msg::AgentState>::SharedPtr agent_state_sub_;
    std::unordered_map<std::string, talk::msg::AgentState> agent_state_table_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
