// Publishes jobs to /incoming_jobs for testing the job assignment... superleader listens for jobs and routes them to the correct zone

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "dirac_msgs/msg/job.hpp"

using namespace std::chrono_literals;

class JobPublisherNode : public rclcpp::Node {
public:
  JobPublisherNode() : Node("job_publisher_node"), job_index_(0) {
    job_pub_ = this->create_publisher<dirac_msgs::msg::Job>("/incoming_jobs", 10);
    // Hardcoded jobs: {job_id, x, y, description}
    jobs_ = {
      {1, 2.0, 3.0, "Job 1"},
      // {2, 5.0, 5.0, "Job 2"},
      // {3, 8.0, 6.0, "Job 3"},
      // {4, 1.0, 9.0, "Job 4"},
      // {5, 3.0, 2.0, "Job 5"}
    };
    timer_ = this->create_wall_timer(2s, std::bind(&JobPublisherNode::publish_job, this));
  }

private:
  void publish_job() {
    if (job_index_ >= jobs_.size()) {
      RCLCPP_INFO(this->get_logger(), "All jobs published.");
      timer_->cancel();
      return;
    }
    auto& j = jobs_[job_index_++];
    dirac_msgs::msg::Job job;
    job.job_id = j.job_id;
    job.x = j.x;
    job.y = j.y;
    job.description = j.description;
    job.assigned_agent_id = -1;
    job.status = "unassigned";
    RCLCPP_INFO(this->get_logger(), "Publishing job %d at (%.1f, %.1f)", job.job_id, job.x, job.y);
    job_pub_->publish(job);
  }

  struct JobData {
    int job_id;
    double x;
    double y;
    std::string description;
  };
  std::vector<JobData> jobs_;
  size_t job_index_;
  rclcpp::Publisher<dirac_msgs::msg::Job>::SharedPtr job_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JobPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
