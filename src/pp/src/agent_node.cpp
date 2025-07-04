#include "rclcpp/rclcpp.hpp"
#include <string>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <vector>
#include <algorithm>
#include "talk/msg/agent_state.hpp"

class AgentNode : public rclcpp::Node{
    public:
        AgentNode(): Node("agent_node")
        {
            //declare params
            declare_parameter("agent_id", "agent1");
            declare_parameter("agent_x", 0.0);
            declare_parameter("agent_y", 0.0);            
            declare_parameter("priority", 0);
            declare_parameter("job_id", "job1");
            declare_parameter("job_x", 0.0);
            declare_parameter("job_y", 0.0);

            //get params
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

            agent_state_pub_=this->create_publisher<talk::msg::AgentState>("agent_state",10)
            
            agent_state_sub_=this->create_subscription<talk::msg::AgentState>("agent_state",10,
                std::bind(&AgentNode::agent_state_callback, this, std::placeholder::_1));
            
        }
private:
        void agent_state_callback(const talk::msg::AgentState::SharedPtr msg){

            agent_state_table_[msg->agent_id] = *msg;

            RCLCPP_INFO(this->get_logger(), "Recieved state from %s", msg->agent_id.c_str());
        }
private:
    void publish_agent_state() {
        // Plan path using A*
        int sx = static_cast<int>(agent_x_);
        int sy = static_cast<int>(agent_y_);
        int gx = static_cast<int>(job_x_);
        int gy = static_cast<int>(job_y_);
        auto path = astar(sx, sy, gx, gy);

        // Prepare next 4 steps (including current position)
        std::vector<std::pair<int, int>> next_steps;
        for (size_t i = 0; i < std::min<size_t>(4, path.size()); ++i) {
            next_steps.push_back(path[i]);
        }
        // If path is shorter than 4, pad with last position
        while (next_steps.size() < 4 && !next_steps.empty()) {
            next_steps.push_back(next_steps.back());
        }

        // Check for conflicts in the next step (step 1)
        bool blocked = false;
        if (next_steps.size() > 1) {
            int nx = next_steps[1].first;
            int ny = next_steps[1].second;
            for (const auto& kv : agent_state_table_) {
                const auto& other = kv.second;
                if (other.agent_id == agent_id_) continue;
                // Only care about higher or equal priority
                if (other.priority >= priority_) {
                    // If other agent's current or next step matches our next step, block
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

        // Only move if not blocked
        if (!blocked && next_steps.size() > 1) {
            agent_x_ = next_steps[1].first;
            agent_y_ = next_steps[1].second;
        }
        // else, stay in place

        // Prepare and publish AgentState message
        talk::msg::AgentState msg;
        msg.agent_id = agent_id_;
        msg.x = agent_x_;
        msg.y = agent_y_;
        msg.priority = priority_;
        msg.job_id = job_id_;
        msg.job_x = job_x_;
        msg.job_y = job_y_;
        // Fill next_steps in the message (assuming AgentState.msg has an array of geometry_msgs/Point called next_steps)
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
    private:
    std::string agent_id_;
    double agent_x_;
    double agent_y_;
    int priority_;
    std::string job_id_;
    double job_x_;
    double job_y_;

    std::unordered_map<std::string, talk::msg::AgentState> agent_state_table_;
    rclcpp::Publisher<talk::msg::AgentState>::SharedPtr agent_state_pub_;
    rclcpp::Subscription<talk::msg::AgentState>::SharedPtr agent_state_sub_;

    // 20x20 warehouse-style map: 0 = free, 1 = obstacle (shelves)
    int map_[20][20] = {
        // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 1
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 3
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 5
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 7
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 8
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 9
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 10
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 11
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 12
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 13
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 14
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 15
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 16
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0}, // 17
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 18
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // 19
    };

        if (curr->x == goal_x && curr->y == goal_y) {
            // Reconstruct path
            Node* n = curr;
            while (n) {
                path.push_back({n->x, n->y});
                n = n->parent;
            }
            std::reverse(path.begin(), path.end());
            // Free memory
            while (!open.empty()) { delete open.top(); open.pop(); }
            return path;
        }

        closed[curr->x][curr->y] = true;

        // 4-connected grid
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
    // No path found
    return {};


rclcpp::TimerBase::SharedPtr timer_;

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
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
