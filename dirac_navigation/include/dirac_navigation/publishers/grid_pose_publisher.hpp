#pragma once

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <dirac_msgs/msg/grid_pose.hpp>
#include <cmath>

namespace dirac_navigation
{
namespace publishers
{

class GridPosePublisher : public rclcpp::Node
{
public:
    explicit GridPosePublisher();

private:
    // Parameters
    double grid_size_;      // Number of cells in each dimension
    double world_size_;     // World size in meters
    double cell_size_;      // Calculated size of each cell
    double origin_x_;       // World origin X
    double origin_y_;       // World origin Y
    double pose_pub_period_sec_; // Period for publishing grid poses
    int32_t agent_id_;      // Agent ID for topic names

    // Last publication time
    rclcpp::Time last_pose_pub_time_;

    // Publishers and Subscribers
    rclcpp::Publisher<dirac_msgs::msg::GridPose>::SharedPtr grid_pose_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

    // Callback for pose subscription
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);

    // Helper functions
    int32_t calculateGridCoordinate(double real_coord, double origin) const;
    double quantizeOrientation(double theta) const;
    void publishGridPose(int32_t grid_x, int32_t grid_y, double grid_theta);
    void loadParameters();
    void setupTopics();
};

} // namespace publishers
} // namespace dirac_navigation 