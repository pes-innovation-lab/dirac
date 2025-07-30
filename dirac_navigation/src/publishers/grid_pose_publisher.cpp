#include "dirac_navigation/publishers/grid_pose_publisher.hpp"

namespace dirac_navigation
{
namespace publishers
{

GridPosePublisher::GridPosePublisher()
    : Node("grid_pose_publisher")
{
    loadParameters();
    setupTopics();
    
    // Calculate cell size based on grid and world size
    cell_size_ = world_size_ / grid_size_;
    
    // Hardcode origin and publishing period
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    pose_pub_period_sec_ = 0.1;  // 10Hz
    
    RCLCPP_INFO(get_logger(), "Grid Pose Publisher initialized for agent %d", agent_id_);
    RCLCPP_INFO(get_logger(), "Grid size: %.1f x %.1f cells", grid_size_, grid_size_);
    RCLCPP_INFO(get_logger(), "World size: %.1f x %.1f meters", world_size_, world_size_);
    RCLCPP_INFO(get_logger(), "Cell size: %.2f meters", cell_size_);
    RCLCPP_INFO(get_logger(), "Origin: (%.1f, %.1f)", origin_x_, origin_y_);
    RCLCPP_INFO(get_logger(), "Publishing rate: %.1f Hz", 1.0/pose_pub_period_sec_);
}

void GridPosePublisher::loadParameters()
{
    // Declare and get parameters with defaults
    this->declare_parameter("grid_size", 10.0);
    this->declare_parameter("world_size", 11.0);  // Turtlesim default world size
    
    // Get agent ID from environment first, then parameter
    const char* agent_id_env = std::getenv("AGENT_ID");
    if (agent_id_env) {
        try {
            agent_id_ = std::stoi(agent_id_env);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Invalid AGENT_ID environment variable, using default: 1");
            agent_id_ = 1;
        }
    } else {
        agent_id_ = 1;
    }
    
    // Load parameters
    grid_size_ = this->get_parameter("grid_size").as_double();
    world_size_ = this->get_parameter("world_size").as_double();
    
    // Initialize last publication time
    last_pose_pub_time_ = this->now();
}

void GridPosePublisher::setupTopics()
{
    // Create publisher for grid pose
    std::string grid_pose_topic = "/robot" + std::to_string(agent_id_) + "/location";
    grid_pose_pub_ = this->create_publisher<dirac_msgs::msg::GridPose>(
        grid_pose_topic, 10);
    
    // Create subscription to real pose
    std::string pose_topic = "/turtle" + std::to_string(agent_id_) + "/pose";
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        pose_topic, 10,
        std::bind(&GridPosePublisher::poseCallback, this, std::placeholders::_1));
        
    RCLCPP_INFO(get_logger(), "Publishing grid poses on: %s", grid_pose_topic.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing to poses from: %s", pose_topic.c_str());
}

int32_t GridPosePublisher::calculateGridCoordinate(double real_coord, double origin) const
{
    return static_cast<int32_t>(std::floor((real_coord - origin) / cell_size_));
}

double GridPosePublisher::quantizeOrientation(double theta) const
{
    // Normalize theta to [0, 2π)
    theta = std::fmod(theta + 2.0 * M_PI, 2.0 * M_PI);
    
    // Quantize to nearest 90 degrees (π/2 radians)
    const double quarter_turn = M_PI / 2.0;
    return std::round(theta / quarter_turn) * quarter_turn;
}

void GridPosePublisher::publishGridPose(int32_t grid_x, int32_t grid_y, double grid_theta)
{
    auto grid_pose = dirac_msgs::msg::GridPose();
    grid_pose.x = grid_x;
    grid_pose.y = grid_y;
    grid_pose.theta = grid_theta;
    grid_pose.grid_size = grid_size_;
    grid_pose.cell_size = cell_size_;
    
    grid_pose_pub_->publish(grid_pose);
    RCLCPP_DEBUG(get_logger(), "Published grid pose: (%d, %d, %.2f) with grid size: %.1f, cell size: %.2f", 
                 grid_x, grid_y, grid_theta, grid_size_, cell_size_);
}

void GridPosePublisher::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    auto now = this->now();
    
    // Check if it's time to publish
    if ((now - last_pose_pub_time_).seconds() >= pose_pub_period_sec_) {
        // Calculate grid coordinates
        int32_t grid_x = calculateGridCoordinate(msg->x, origin_x_);
        int32_t grid_y = calculateGridCoordinate(msg->y, origin_y_);
        double grid_theta = quantizeOrientation(msg->theta);
        
        // Publish grid pose
        publishGridPose(grid_x, grid_y, grid_theta);
        
        // Update last publication time
        last_pose_pub_time_ = now;
    }
}

} // namespace publishers
} // namespace dirac_navigation 