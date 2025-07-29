#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dirac_lib/election.hpp"
#include "dirac_msgs/msg/job.hpp"
#include "dirac_msgs/msg/path.hpp"

class ElectionHandler {
public: 
    double MAP_SIZE = 5.0; 
    int ZONES_PER_ROW = 1; 
    int zone_id;
    
    ElectionHandler(int agent_id, int zone_id, double agent_x, double agent_y, rclcpp::Node::SharedPtr node)
    {
        election_manager_ = std::make_shared<dirac_lib::ElectionManager>(
            agent_id, zone_id, agent_x, agent_y, node);
    }

    void start()
    {
        election_manager_->startElection();
    }

private:
    std::shared_ptr<dirac_lib::ElectionManager> election_manager_;
};


class Agent : public std::enable_shared_from_this<Agent>
{
public:
    double MAP_SIZE = 5.0;
    int ZONES_PER_ROW = 1;
    int zone_id;

    static std::shared_ptr<Agent> create(const rclcpp::Node::SharedPtr& node)
    {
        return std::shared_ptr<Agent>(new Agent(node));
    }

    void init()
    {
        // Initialize publishers and subscribers for job and path
        job_pub_ = node_->create_publisher<dirac_msgs::msg::Job>("job", 10);
        path_sub_ = node_->create_subscription<dirac_msgs::msg::Path>(
            "planned_path", 10,
            std::bind(&Agent::pathCallback, this, std::placeholders::_1)
        );

        election_handler_ = std::make_shared<ElectionHandler>(
            agent_id_, zone_id_, agent_x_, agent_y_, node_);

        election_handler_->start();
    }

    rclcpp::Node::SharedPtr getNode() const { return node_; }

    // Function to publish a job (to be called from agent_node.cpp)
    void publishJob(const dirac_msgs::msg::Job &job_msg)
    {
        if (job_pub_) job_pub_->publish(job_msg);
    }

    // Callback for receiving planned paths
    void pathCallback(const dirac_msgs::msg::Path::SharedPtr msg)
    {
        last_path_ = *msg;
        // Further processing can be added here
    }

    // Accessor for last received path
    dirac_msgs::msg::Path getLastPath() const
    {
        return last_path_;
    }

    // Calculate which zone the agent is in based on its coordinates
    int calculateZoneId(double x, double y) const
    {
        int row = static_cast<int>(y / (MAP_SIZE / ZONES_PER_ROW));
        int col = static_cast<int>(x / (MAP_SIZE / ZONES_PER_ROW));
        return row * ZONES_PER_ROW + col;
    }

private:
    Agent(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        is_leader_ = node->get_parameter("isLeader").as_bool();
        agent_id_ = node->get_parameter("agent_id").as_int();
        zone_id_ = node->get_parameter("zone_id").as_int();
        agent_x_ = node->get_parameter("agent_x").as_double();
        agent_y_ = node->get_parameter("agent_y").as_double();
        z_leader = node->get_parameter("z_leader").as_int();
    }

    rclcpp::Node::SharedPtr node_;
    int agent_id_, zone_id_;
    double agent_x_, agent_y_;
    bool is_leader_;
    int z_leader;

    std::shared_ptr<ElectionHandler> election_handler_;

    // Publisher and subscriber for job and path
    rclcpp::Publisher<dirac_msgs::msg::Job>::SharedPtr job_pub_;
    rclcpp::Subscription<dirac_msgs::msg::Path>::SharedPtr path_sub_;

    // Store last received path
    dirac_msgs::msg::Path last_path_;
};
