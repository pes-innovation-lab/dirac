#include "dirac_navigation/publishers/grid_pose_publisher.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dirac_navigation::publishers::GridPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 