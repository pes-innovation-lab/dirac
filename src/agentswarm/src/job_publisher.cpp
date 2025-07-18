#include "rclcpp/rclcpp.hpp"
#include "agentswarm/msg/job_assignment.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("job_publisher");
    auto pub = node->create_publisher<agentswarm::msg::JobAssignment>("/incoming_jobs", 10);

    struct JobCoord {int id; float x; float y;}; // > job struct for id and coords
    std::vector<JobCoord> jobs = {
        {6,25,10},
        {5,22,15},
        {9,24,18},
        {15, 30, 30},
    };

    rclcpp::sleep_for(std::chrono::seconds(1)); // -> wait for subs to connect

    for (const auto& job : jobs) {
        auto msg = agentswarm::msg::JobAssignment();
        msg.job_id = job.id;
        msg.x = job.x;
        msg.y = job.y;
        msg.zone_id = -1;
        msg.assigned_agent_id = -1;
        msg.completed = false;
        RCLCPP_INFO(node->get_logger(), "Publishing job %d at (%.2f, %.2f)", msg.job_id, msg.x, msg.y);
        pub->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(2)); // -> space out job publishing
    }

    rclcpp::shutdown();
    return 0;
}
