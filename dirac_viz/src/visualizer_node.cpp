#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

struct AgentInfo {
    int id;
    int x, y;
    int goal_x, goal_y;
    int priority;
    std::string job_id;
    bool is_leader;
};

struct PID {
    double kp, ki, kd;
    double prev_error_x = 0, prev_error_y = 0;
    double integral_x = 0, integral_y = 0;
    PID(double p, double i, double d) : kp(p), ki(i), kd(d) {}
    PID() : kp(1.0), ki(0.0), kd(0.0) {} // Default constructor
    std::pair<double, double> step(double error_x, double error_y, double dt) {
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        double derivative_x = (error_x - prev_error_x) / dt;
        double derivative_y = (error_y - prev_error_y) / dt;
        prev_error_x = error_x;
        prev_error_y = error_y;
        double vx = kp * error_x + ki * integral_x + kd * derivative_x;
        double vy = kp * error_y + ki * integral_y + kd * derivative_y;
        return {vx, vy};
    }
};

class GridVisualizer : public rclcpp::Node {
public:
    GridVisualizer() : Node("grid_visualizer") {
        declare_parameter<std::string>("map_csv", "map.csv");
        declare_parameter<std::string>("agents_csv", "agents.csv");
        get_parameter("map_csv", map_csv_);
        get_parameter("agents_csv", agents_csv_);
        
        load_map();
        load_agents();
        
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        
        // Subscribe to each agent's direction topic
        for (const auto& [id, agent] : agents_) {
            std::string topic = "/agent_command_" + std::to_string(id);
            auto sub = create_subscription<dirac_msgs::msg::AgentCommand>(
                topic, 10,
                [this, id](const dirac_msgs::msg::AgentCommand::SharedPtr msg) {
                    this->on_direction(id, msg->direction);
                }
            );
            agent_subs_.push_back(sub);
        }
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        for (const auto& [id, agent] : agents_) {
            agent_pose_[id] = {static_cast<double>(agent.x), static_cast<double>(agent.y)};
            agent_target_cell_[id] = {agent.x, agent.y};
            agent_pid_[id] = PID(2.0, 0.0, 0.2); // Tune as needed
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publish_markers(); });
    }

private:
    void load_map() {
        std::ifstream file(map_csv_);
        std::string line;
        while (std::getline(file, line)) {
            std::vector<int> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                row.push_back(std::stoi(cell));
            }
            map_.push_back(row);
        }
    }
    void load_agents() {
        std::ifstream file(agents_csv_);
        std::string line;
        std::getline(file, line); // skip header
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string cell;
            AgentInfo agent;
            std::getline(ss, cell, ','); agent.id = std::stoi(cell);
            std::getline(ss, cell, ','); agent.x = std::stoi(cell);
            std::getline(ss, cell, ','); agent.y = std::stoi(cell);
            std::getline(ss, cell, ','); agent.priority = std::stoi(cell);
            std::getline(ss, cell, ','); agent.job_id = cell;
            std::getline(ss, cell, ','); agent.goal_x = std::stoi(cell);
            std::getline(ss, cell, ','); agent.goal_y = std::stoi(cell);
            std::getline(ss, cell, ','); agent.is_leader = (cell == "true" || cell == "1");
            agents_[agent.id] = agent;
            agent_positions_[agent.id] = {agent.x, agent.y};
            job_positions_[agent.id] = {agent.goal_x, agent.goal_y};
        }
    }
    void on_direction(int agent_id, int direction) {
        // 1 = forward (up), 2 = backward (down), 3 = left, 4 = right
        auto& target = agent_target_cell_[agent_id];
        int x = target.first, y = target.second;
        switch (direction) {
            case 1: y -= 1; break; // up
            case 2: y += 1; break; // down
            case 3: x -= 1; break; // left
            case 4: x += 1; break; // right
            default: break;
        }
        // Check bounds and obstacles
        if (y >= 0 && y < (int)map_.size() && x >= 0 && x < (int)map_[0].size() && map_[y][x] == 0) {
            target = {x, y};
        }
    }
    void publish_markers() {
        double dt = 0.5; // seconds (matches timer period)
        visualization_msgs::msg::MarkerArray marker_array;
        // Draw grid
        int id = 0;
        for (size_t y = 0; y < map_.size(); ++y) {
            for (size_t x = 0; x < map_[y].size(); ++x) {
                visualization_msgs::msg::Marker cell;
                cell.header.frame_id = "map";
                cell.header.stamp = now();
                cell.ns = "grid";
                cell.id = id++;
                cell.type = visualization_msgs::msg::Marker::CUBE;
                cell.action = visualization_msgs::msg::Marker::ADD;
                cell.pose.position.x = x;
                cell.pose.position.y = y;
                cell.pose.position.z = 0;
                cell.scale.x = 1.0;
                cell.scale.y = 1.0;
                cell.scale.z = 0.05;
                if (map_[y][x] == 0) {
                    cell.color.r = 0.9f; cell.color.g = 0.9f; cell.color.b = 0.9f; cell.color.a = 0.3f;
                } else {
                    cell.color.r = 0.2f; cell.color.g = 0.2f; cell.color.b = 0.2f; cell.color.a = 0.7f;
                }
                marker_array.markers.push_back(cell);
            }
        }
        // Draw agents and broadcast TF
        for (const auto& [id, target_cell] : agent_target_cell_) {
            // PID update
            auto& pose = agent_pose_[id];
            auto& pid = agent_pid_[id];
            double error_x = target_cell.first - pose.first;
            double error_y = target_cell.second - pose.second;
            auto [vx, vy] = pid.step(error_x, error_y, dt);
            // Restrict to one axis per tick (Manhattan movement)
            if (std::abs(error_x) > std::abs(error_y)) {
                vy = 0.0;
            } else if (std::abs(error_y) > std::abs(error_x)) {
                vx = 0.0;
            } else if (std::abs(error_x) > 0.0) {
                // If both errors are equal and nonzero, prefer x axis
                vy = 0.0;
            } else {
                vx = 0.0;
                vy = 0.0;
            }
            // Limit max speed for realism
            double max_speed = 2.0 * dt;
            if (vx > max_speed) vx = max_speed;
            if (vx < -max_speed) vx = -max_speed;
            if (vy > max_speed) vy = max_speed;
            if (vy < -max_speed) vy = -max_speed;
            pose.first += vx;
            pose.second += vy;
            // Draw agent marker at pose
            visualization_msgs::msg::Marker agent_marker;
            agent_marker.header.frame_id = "map";
            agent_marker.header.stamp = now();
            agent_marker.ns = "agents";
            agent_marker.id = id;
            agent_marker.type = visualization_msgs::msg::Marker::SPHERE;
            agent_marker.action = visualization_msgs::msg::Marker::ADD;
            agent_marker.pose.position.x = pose.first;
            agent_marker.pose.position.y = pose.second;
            agent_marker.pose.position.z = 0.3;
            agent_marker.scale.x = 0.7;
            agent_marker.scale.y = 0.7;
            agent_marker.scale.z = 0.7;
            agent_marker.color.r = 0.1f; agent_marker.color.g = 0.2f + 0.7f * (id % 2); agent_marker.color.b = 0.8f; agent_marker.color.a = 1.0f;
            marker_array.markers.push_back(agent_marker);
            // Broadcast TF
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now();
            t.header.frame_id = "map";
            t.child_frame_id = "agent_" + std::to_string(id);
            t.transform.translation.x = pose.first;
            t.transform.translation.y = pose.second;
            t.transform.translation.z = 0.0;
            t.transform.rotation.w = 1.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            tf_broadcaster_->sendTransform(t);
        }
        // Draw jobs (goals)
        for (const auto& [id, pos] : job_positions_) {
            visualization_msgs::msg::Marker job_marker;
            job_marker.header.frame_id = "map";
            job_marker.header.stamp = now();
            job_marker.ns = "jobs";
            job_marker.id = id;
            job_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            job_marker.action = visualization_msgs::msg::Marker::ADD;
            job_marker.pose.position.x = pos.first;
            job_marker.pose.position.y = pos.second;
            job_marker.pose.position.z = 0.1;
            job_marker.scale.x = 0.5;
            job_marker.scale.y = 0.5;
            job_marker.scale.z = 0.2;
            job_marker.color.r = 1.0f; job_marker.color.g = 0.7f; job_marker.color.b = 0.2f; job_marker.color.a = 1.0f;
            marker_array.markers.push_back(job_marker);
        }
        marker_pub_->publish(marker_array);
    }

    std::string map_csv_;
    std::string agents_csv_;
    std::vector<std::vector<int>> map_;
    std::unordered_map<int, AgentInfo> agents_;
    std::unordered_map<int, std::pair<int, int>> agent_positions_;
    std::unordered_map<int, std::pair<int, int>> job_positions_;
    std::vector<rclcpp::Subscription<dirac_msgs::msg::AgentCommand>::SharedPtr> agent_subs_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // For each agent: current pose (float), target cell (int,int), and PID
    std::unordered_map<int, std::pair<double, double>> agent_pose_; // x, y (float)
    std::unordered_map<int, std::pair<int, int>> agent_target_cell_;
    std::unordered_map<int, PID> agent_pid_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
