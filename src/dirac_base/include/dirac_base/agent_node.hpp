#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dirac_lib/election.hpp"

class ElectionHandler {
public: 
    double MAP_SIZE = 30.0; 
    int ZONES_PER_ROW = 3; 
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
    double MAP_SIZE = 30.0;
    int ZONES_PER_ROW = 3;
    int zone_id_;

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
        agent_x_ = node->get_parameter("agent_x").as_double();
        agent_y_ = node->get_parameter("agent_y").as_double();
        z_leader = node->get_parameter("z_leader").as_int();

        //  Calculate zone_id dynamically
        zone_id_ = calculate_zone(agent_x_, agent_y_);
        if (zone_id_ == -1)
        {
            RCLCPP_ERROR(node_->get_logger(), "Agent position is out of map bounds!");
        }
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

    rclcpp::Node::SharedPtr node_;
    int agent_id_;
    double agent_x_, agent_y_;
    bool is_leader_;
    int z_leader;

    std::shared_ptr<ElectionHandler> election_handler_;
};
