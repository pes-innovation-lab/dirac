// job_assignment.hpp
// Library interface for job assignment and routing in DIRAC
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <dirac_msgs/msg/job.hpp>
#include <dirac_msgs/msg/job_bid.hpp>
#include <dirac_msgs/msg/job_completion.hpp>
#include <functional>
#include <memory>
#include <string>

namespace dirac_lib {

// Utility function to determine zone from coordinates
int determine_zone(double x, double y);

// Super Leader: Listens for jobs and routes to correct zone
class JobSuperLeader {
public:

  JobSuperLeader(const rclcpp::Node::SharedPtr& node);
private:
  void incoming_job_callback(const dirac_msgs::msg::Job::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<dirac_msgs::msg::Job>::SharedPtr job_sub_;
  std::map<int, rclcpp::Publisher<dirac_msgs::msg::Job>::SharedPtr> zone_publishers_;
};

// Zone Leader: Handles jobs for a zone, collects bids, assigns jobs (sets assigned_agent_id and status)
class JobZoneLeader {
public:
  JobZoneLeader(const rclcpp::Node::SharedPtr& node, int zone_id);
private:
  void zone_job_callback(const dirac_msgs::msg::Job::SharedPtr msg);
  void bid_callback(const dirac_msgs::msg::JobBid::SharedPtr msg);
  void timer_callback();
  rclcpp::Node::SharedPtr node_;
  int zone_id_;
  rclcpp::Subscription<dirac_msgs::msg::Job>::SharedPtr zone_job_sub_;
  rclcpp::Publisher<dirac_msgs::msg::Job>::SharedPtr jobs_pub_;
  rclcpp::Subscription<dirac_msgs::msg::JobBid>::SharedPtr bid_sub_;
  rclcpp::TimerBase::SharedPtr bid_timer_;
  std::vector<dirac_msgs::msg::JobBid> current_bids_;
  dirac_msgs::msg::Job current_job_;
  bool collecting_bids_ = false;
};

// Agent: Listens for jobs, bids if available (checks assigned_agent_id and status fields)
class JobAgent {
public:
  JobAgent(const rclcpp::Node::SharedPtr& node, int agent_id, int zone_id);
  void set_busy(bool busy);
private:
  void job_callback(const dirac_msgs::msg::Job::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  int agent_id_;
  int zone_id_;
  double agent_x_ = 0.0;
  double agent_y_ = 0.0;
  bool busy_ = false;
  rclcpp::Subscription<dirac_msgs::msg::Job>::SharedPtr jobs_sub_;
  rclcpp::Publisher<dirac_msgs::msg::JobBid>::SharedPtr bid_pub_;
};

} // namespace dirac_lib
