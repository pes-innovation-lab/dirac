#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dirac_lib/election.hpp"

class ElectionHandler {
public: 
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
    static std::shared_ptr<Agent> create(const rclcpp::Node::SharedPtr& node)
    {
        return std::shared_ptr<Agent>(new Agent(node));
    }

    void init()
    {
        election_handler_ = std::make_shared<ElectionHandler>(
            agent_id_, zone_id_, agent_x_, agent_y_, node_);

        election_handler_->start();  
    }

    rclcpp::Node::SharedPtr getNode() const { return node_; }

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
};
