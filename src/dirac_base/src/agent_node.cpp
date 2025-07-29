#include "rclcpp/rclcpp.hpp"
#include "dirac_base/agent_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("agent_node");

    // Declare parameters
    node->declare_parameter<bool>("isLeader", false);

    node->declare_parameter<int>("agent_id", 0);
    node->declare_parameter<int>("zone_id", 0);
    node->declare_parameter<double>("agent_x", 0.0);
    node->declare_parameter<double>("agent_y", 0.0);
    node->declare_parameter<int>("z_leader", -1);

    // Create agent and initialize logic
    auto agent = Agent::create(node);
    agent->init();  

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}