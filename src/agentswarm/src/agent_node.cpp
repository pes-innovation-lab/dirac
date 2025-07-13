#include <string>
#include <cmath>     // For std::floor
#include <algorithm> // For std::min
#include <vector>
#include <map>
#include <chrono>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Define map and zone constants for clarity
const double MAP_SIZE = 30.0;
const int ZONES_PER_ROW = 3;

class AgentNode : public rclcpp::Node
{
public:
    AgentNode() : Node("agent_node") // The launch file will override this name
    {
        this->declare_parameter<double>("pos_x", 0.0);
        this->declare_parameter<double>("pos_y", 0.0);
        this->declare_parameter<int>("agent_id", 0);
        this->declare_parameter<int>("zone_id",0);
        this->declare_parameter<int>("z_leader",0);
        this->declare_parameter<bool>("isLeader",0);
        double pos_x = this->get_parameter("pos_x").as_double();
        double pos_y = this->get_parameter("pos_y").as_double();
        agent_id_ = this->get_parameter("agent_id").as_int();
        int zone = calculate_zone(pos_x, pos_y);
        this->set_parameter(rclcpp::Parameter("zone_id", zone));
        zone_id_ = zone;
        pos_x_ = pos_x;
        pos_y_ = pos_y;

        std::string z_topic= "/zone_" + std::to_string(zone) + "/election";
        election_ = this->create_subscription<std_msgs::msg::String>(
            z_topic,10,std::bind(&AgentNode::election_function,this,std::placeholders::_1));
        election_pub_ = this->create_publisher<std_msgs::msg::String>(z_topic, 10);

        // Calculate center of zone
        get_zone_center(zone_id_, center_x_, center_y_);
        distance_to_center_ = std::sqrt(std::pow(pos_x_ - center_x_, 2) + std::pow(pos_y_ - center_y_, 2));

        // Publish our info after a short delay to allow all nodes to start
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&AgentNode::publish_own_info, this)
        );

        // Election timer (wait for all messages, then elect leader)
        election_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&AgentNode::run_election, this)
        );

        RCLCPP_INFO(this->get_logger(), 
            "Agent %d at (x: %.2f, y: %.2f) and belongs to Zone %d. Subscribing to topic %s",
            agent_id_, pos_x_, pos_y_, zone_id_, z_topic.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr election_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr election_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr election_timer_;

    int agent_id_;
    int zone_id_;
    double pos_x_, pos_y_;
    double center_x_, center_y_;
    double distance_to_center_;

    // Map: agent_id -> distance
    std::map<int, double> agent_distances_;
    bool election_done_ = false;

    void get_zone_center(int zone_id, double &cx, double &cy)
    {
        int row = (zone_id - 1) / 3;
        int col = (zone_id - 1) % 3;
        double zone_size = 10.0;
        cx = col * zone_size + zone_size / 2.0;
        cy = row * zone_size + zone_size / 2.0;
    }

    void publish_own_info()
    {
        // Publish our info as a string: "agent_id distance"
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << agent_id_ << " " << distance_to_center_;
        msg.data = oss.str();
        election_pub_->publish(msg);
    }

    void election_function(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse: "agent_id distance"
        std::istringstream iss(msg->data);
        int other_id;
        double other_dist;
        if (iss >> other_id >> other_dist) {
            agent_distances_[other_id] = other_dist;
        }
    }

    void run_election()
    {
        if (election_done_) return;
        election_done_ = true;

        // Add self if not already in map
        agent_distances_[agent_id_] = distance_to_center_;

        // Find leader (min distance, tie: min agent_id)
        int leader_id = agent_id_;
        double min_dist = distance_to_center_;
        for (const auto& kv : agent_distances_) {
            if (kv.second < min_dist || (kv.second == min_dist && kv.first < leader_id)) {
                min_dist = kv.second;
                leader_id = kv.first;
            }
        }
        bool is_leader = (agent_id_ == leader_id);

        // Set parameters
        this->set_parameter(rclcpp::Parameter("isLeader", is_leader));
        this->set_parameter(rclcpp::Parameter("z_leader", leader_id));

        // Log leader election result
        if (is_leader) {
            RCLCPP_INFO(this->get_logger(), "I am the leader for zone %d (agent %d)", zone_id_, agent_id_);
        } else {
            RCLCPP_INFO(this->get_logger(), "I am NOT the leader for zone %d. Leader is agent %d", zone_id_, leader_id);
        }

        // // Log state of all agents in this zone
        // std::ostringstream oss;
        // oss << "Zone " << zone_id_ << " state after election: ";
        // for (const auto& kv : agent_distances_) {
        //     oss << "[agent " << kv.first << ": dist=" << kv.second;
        //     if (kv.first == leader_id) oss << " LEADER";
        //     oss << "] ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    int calculate_zone(double x, double y)
    {
        if (x < 0 || x > MAP_SIZE || y < 0 || y > MAP_SIZE)
        {
            return -1; // Out of map bounds
        }
        double zone_size = MAP_SIZE / ZONES_PER_ROW;
        int col_index = static_cast<int>(std::floor(x / zone_size));
        int row_index = static_cast<int>(std::floor(y / zone_size));
        col_index = std::min(col_index, ZONES_PER_ROW - 1);
        row_index = std::min(row_index, ZONES_PER_ROW - 1);
        int zone_number = row_index * ZONES_PER_ROW + col_index + 1;
        return zone_number;
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);
    
    // Create an instance of our AgentNode and spin it.
    // Since all logic is in the constructor, it will execute and print immediately.
    // The node will then wait for shutdown (e.g., Ctrl+C).
    rclcpp::spin(std::make_shared<AgentNode>());
    
    // Shutdown the ROS2 system
    rclcpp::shutdown();
    
    return 0;
}