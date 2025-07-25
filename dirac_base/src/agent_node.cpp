#include <rclcpp/rclcpp.hpp>
#include "dirac_lib/election.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("agent_node");

  // Declare parameters with default values
  node->declare_parameter<int>("agent_id", 1);
  node->declare_parameter<int>("zone_id", 1);
  node->declare_parameter<double>("agent_x", 10.0);
  node->declare_parameter<double>("agent_y", 20.0);
  // Declare leader election result parameters
  node->declare_parameter<bool>("isLeader", false);
  node->declare_parameter<int>("z_leader", -1);

  // Get parameters
  int agent_id = node->get_parameter("agent_id").as_int();
  int zone_id = node->get_parameter("zone_id").as_int();
  double agent_x = node->get_parameter("agent_x").as_double();
  double agent_y = node->get_parameter("agent_y").as_double();

  dirac_lib::ElectionManager election_manager(agent_id, zone_id, agent_x, agent_y, node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
