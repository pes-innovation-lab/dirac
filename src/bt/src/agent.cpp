#include <cstdio>
#include "rclcpp/rclcpp.hpp"

class Agent : public rclcpp::Node {
public:
    Agent() : Node("agent") {
        // Node initialization
    }
};

int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  printf("hello world bt package\n");
  return 0;
}
