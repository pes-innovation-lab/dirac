#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "talk/msg/agent_state.hpp"
#include "talk/msg/agent_state_table.hpp"

#include <string>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <vector>
#include <algorithm>

#define MAP_SIZE 20

struct cell {
    int x;
    int y;
    double f; // total cost
    double g; // cost from start to current cell
    double h; // heuristic cost to goal
};

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

class AgentNode : public rclcpp::Node {
public:
    AgentNode() : Node("agent_node") {
        // declare & get parameters
        declare_parameter("agent_id", "agent1");
        declare_parameter("agent_x", 0);
        declare_parameter("agent_y", 0);            
        declare_parameter("priority", 0);
        declare_parameter("job_id", "job1");
        declare_parameter("job_x", 0);
        declare_parameter("job_y", 0);

        agent_id_ = get_parameter("agent_id").as_string();  
        agent_x_ = get_parameter("agent_x").as_int();
        agent_y_ = get_parameter("agent_y").as_int();
        priority_ = get_parameter("priority").as_int();
        job_id_ = get_parameter("job_id").as_string();
        job_x_ = get_parameter("job_x").as_int();
        job_y_ = get_parameter("job_y").as_int();
        

        publisher_ = this->create_publisher<talk::msg::AgentState>("agent_state", 10);
        timer=this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&AgentNode::publish_state_callback, this);
        );
        RCLCPP_INFO(this->get_logger(), "Agent Node initialized with ID: %s at (%d, %d) with priority %d, job %s at (%d, %d)",
                    agent_id_.c_str(), agent_x_, agent_y_, priority_, job_id_.c_str(), job_x_, job_y_);
        subscriber_ = this->create_subscription<talk::msg::AgentStateTable>(
            "agent_state_table",
            10,
            std::bind(&AgentNode::subscribe_table_callback, this, std::placeholders::_1);
        );
        agent_states_.clear();
    
    }
    void publish_state_callback();
private:
    std::string agent_id_;
    int agent_x_;
    int agent_y_;
    int priority_;
    std::string job_id_;
    int job_x_;
    int job_y_;
    rclcpp::Publisher<talk::msg::AgentState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<talk::msg::AgentStateTable>::SharedPtr subscriber_;
    std::unordered_map<std::string, talk::msg::AgentState> agent_states_; // agent_id -> state
    void subscribe_table_callback(const talk::msg::AgentStateTable::SharedPtr msg) {
        for (const auto& state : msg->agent_states) {
            agent_states_[state.agent_id] = state;
        }
    }

    bool unblocked(int x, int y){
        if(map_[x][y]==0){
            return true;
        }
        else{
            return false;
        }
    }

    bool isDestination(int x, int y, int goal_x, int goal_y) {
        return (x == goal_x && y == goal_y);
    }

    cell findAdjacentPoints(int x, int y){
        std::vector<cell> adjacent_points;
        // left
        if (x > 0 && map_[x-1][y] == 0) {
            adjacent_points.push_back(cell{x-1, y, 0, 0, 0});
        }
        // up
        if (y < MAP_SIZE-1 && map_[x][y+1] == 0) {
            adjacent_points.push_back(cell{x, y+1, 0, 0, 0});
        }
        // right
        if (x < MAP_SIZE-1 && map_[x+1][y] == 0) {
            adjacent_points.push_back(cell{x+1, y, 0, 0, 0});
        }
        // down
        if (y > 0 && map_[x][y-1] == 0) {
            adjacent_points.push_back(cell{x, y-1, 0, 0, 0});
        }
        return adjacent_points;
    }

    double findH(const cell& point, int goal_x, int goal_y){
        return std::sqrt(std::pow(point.x - goal_x, 2) + std::pow(point.y - goal_y, 2));
    }
    double findG(const cell& point, int start_x, int start_y){
        return std::abs(point.x - start_x) + std::abs(point.y - start_y);
    }
    double findF(const cell& point, int start_x, int start_y, int goal_x, int goal_y){
        return findG(point, start_x, start_y) + findH(point, goal_x, goal_y);
    }

    cell Astar(int start_x, int start_y, int goal_x, int goal_y){
        if(start_x < 0 || start_x >= MAP_SIZE || start_y < 0 || start_y >= MAP_SIZE ||
           goal_x < 0 || goal_x >= MAP_SIZE || goal_y < 0 || goal_y >= MAP_SIZE) {
            RCLCPP_ERROR(this->get_logger(), "Start or goal position out of bounds");
            return cell{start_x, start_y, 0, 0, 0};
        }
        if(!unblocked(start_x, start_y) || !unblocked(goal_x, goal_y)) {
            RCLCPP_ERROR(this->get_logger(), "Start position or goal position is blocked");
            return cell{start_x, start_y, 0, 0, 0};
        }
        if(isDestination(start_x, start_y, goal_x, goal_y)) {
            RCLCPP_INFO(this->get_logger(), "Already at the destination");
            return cell{start_x, start_y, 0, 0, 0};
        }
        std::vector<cell> adjacent_points = findAdjacentPoints(start_x, start_y);
        if(adjacent_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No adjacent points found");
            return cell{start_x, start_y, 0, 0, 0};
        }
        cell* best = nullptr;
        double best_f = std::numeric_limits<double>::max();
        for(auto& pt : adjacent_points){
            pt.g = 1; // cost from start to adjacent is always 1
            pt.h = findH(pt, goal_x, goal_y);
            pt.f = pt.g + pt.h;
            if(pt.f < best_f){
                best_f = pt.f;
                best = &pt;
            }
        }
        if(best) return *best;
        return cell{start_x, start_y, 0, 0, 0};
    }
};

AgentNode::publish_state_callback(){
    talk::msg::AgentState msg;
    msg.agent_id = agent_id_;
    msg.x = agent_x_;
    msg.y = agent_y_;
    msg.priority = priority_;
    msg.job_id = job_id_;        
    msg.job_x = job_x_;
    msg.job_y = job_y_;
    msg.next_steps.clear();
    msg.next_steps.reserve(4);
    int cur_x = msg.x;
    int cur_y = msg.y;
    for(int i=0; i<4; i++){
        cell next = Astar(cur_x, cur_y, msg.job_x, msg.job_y);
        // Priority-based conflict check
        bool conflict = false;
        for (const auto& [other_id, other_state] : agent_states_) {
            if (other_id == agent_id_) continue;
            if (other_state.priority > msg.priority) {
                // Check if higher-priority agent plans to move to this cell
                for (const auto& pt : other_state.next_steps) {
                    if ((int)pt.x == next.x && (int)pt.y == next.y) {
                        conflict = true;
                        break;
                    }
                }
            }
            if (conflict) break;
        }
        if (conflict) {
            // Wait in place if conflict
            next.x = cur_x;
            next.y = cur_y;
        }
        geometry_msgs::msg::Point pt;
        pt.x = next.x;
        pt.y = next.y;
        pt.z = 0;
        msg.next_steps.push_back(pt);
        cur_x = next.x;
        cur_y = next.y;
    }
    talk::msg::AgentStateTable msg_table;
    msg_table.agent_states.push_back(msg);
    publisher_->publish(msg_table);
    RCLCPP_INFO(this->get_logger(), "Published agent state: %s at (%d, %d) with priority %d, job %s at (%d, %d)",
                msg.agent_id.c_str(), msg.x, msg.y, msg.priority, msg.job_id.c_str(), msg.job_x, msg.job_y);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);   
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
