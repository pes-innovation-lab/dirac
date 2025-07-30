// job_assignment.cpp
// Implementation of job assignment and routing library for DIRAC

#include "dirac_lib/job_assignment.hpp"
#include <map>
#include <string>
#include <vector>
#include <algorithm>

namespace dirac_lib {

// Example: Simple zone calculation (customize as needed)
int determine_zone(double x, double y) {
  // For demo: 9 zones in a 30x30 grid (3x3) //make this a part of agent class where the zone is determined
  if (x < 10 && y < 10) return 1;
  if (x >= 10 && x < 20 && y < 10) return 2;
  if (x >= 20 && y < 10) return 3;
  if (x < 10 && y >= 10 && y < 20) return 4;
  if (x >= 10 && x < 20 && y >= 10 && y < 20) return 5;
  if (x >= 20 && y >= 10 && y < 20) return 6;
  if (x < 10 && y >= 20) return 7;
  if (x >= 10 && x < 20 && y >= 20) return 8;
  return 9;
}


JobSuperLeader::JobSuperLeader(const rclcpp::Node::SharedPtr& node) : node_(node) {
  job_sub_ = node_->create_subscription<dirac_msgs::msg::Job>(
    "/incoming_jobs", 20,
    std::bind(&JobSuperLeader::incoming_job_callback, this, std::placeholders::_1));
}

void JobSuperLeader::incoming_job_callback(const dirac_msgs::msg::Job::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "SuperLeader received job %d at (%.2f, %.2f)", msg->job_id, msg->x, msg->y);
  int zone = determine_zone(msg->x, msg->y);
  RCLCPP_INFO(node_->get_logger(), "SuperLeader determined zone %d for job %d", zone, msg->job_id);
  if (zone <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "SuperLeader: Invalid zone %d for job %d! Job will not be routed.", zone, msg->job_id);
    return;
  }
  std::string topic = "/zone_" + std::to_string(zone) + "/incoming_jobs";
  if (zone_publishers_.find(zone) == zone_publishers_.end()) {
    zone_publishers_[zone] = node_->create_publisher<dirac_msgs::msg::Job>(topic, 10);
    RCLCPP_INFO(node_->get_logger(), "SuperLeader created publisher for topic: %s", topic.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "SuperLeader reusing publisher for topic: %s", topic.c_str());
  }
  if (!zone_publishers_[zone]) {
    RCLCPP_ERROR(node_->get_logger(), "SuperLeader: Publisher for zone %d (topic %s) is null!", zone, topic.c_str());
    return;
  }
  zone_publishers_[zone]->publish(*msg);
  RCLCPP_INFO(node_->get_logger(), "SuperLeader routed job %d to zone %d on topic %s (job pos: %.2f, %.2f)",
              msg->job_id, zone, topic.c_str(), msg->x, msg->y);
}

JobZoneLeader::JobZoneLeader(const rclcpp::Node::SharedPtr& node, int zone_id)
  : node_(node), zone_id_(zone_id) {
  std::string incoming_topic = "/zone_" + std::to_string(zone_id_) + "/incoming_jobs";
  zone_job_sub_ = node_->create_subscription<dirac_msgs::msg::Job>(
    incoming_topic, 10,
    std::bind(&JobZoneLeader::zone_job_callback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "ZoneLeader for zone %d subscribed to %s", zone_id_, incoming_topic.c_str());
  std::string bid_topic = "/zone_" + std::to_string(zone_id_) + "/job_bids";
  bid_sub_ = node_->create_subscription<dirac_msgs::msg::JobBid>(
    bid_topic, 10,
    std::bind(&JobZoneLeader::bid_callback, this, std::placeholders::_1));
  std::string jobs_topic = "/zone_" + std::to_string(zone_id_) + "/jobs";
  jobs_pub_ = node_->create_publisher<dirac_msgs::msg::Job>(jobs_topic, 10);
}

void JobZoneLeader::zone_job_callback(const dirac_msgs::msg::Job::SharedPtr msg) {
  if (collecting_bids_) return; // Ignore if already collecting bids
  RCLCPP_INFO(node_->get_logger(), "ZoneLeader %d received job %d at (%.1f, %.1f)", zone_id_, msg->job_id, msg->x, msg->y);
  // Start collecting bids for this job
  current_job_ = *msg;
  current_bids_.clear();
  collecting_bids_ = true;
  // -> Broadcasting jobs like a radio DJ 
  jobs_pub_->publish(*msg); // Broadcast to agents
  RCLCPP_INFO(node_->get_logger(), "ZoneLeader %d broadcasting job %d", zone_id_, msg->job_id);
  bid_timer_ = node_->create_wall_timer(
    std::chrono::seconds(2), std::bind(&JobZoneLeader::timer_callback, this));
}

void JobZoneLeader::bid_callback(const dirac_msgs::msg::JobBid::SharedPtr msg) {
  if (!collecting_bids_) return;
  if (msg->job_id == current_job_.job_id) {
    current_bids_.push_back(*msg);
    RCLCPP_INFO(node_->get_logger(), "ZoneLeader %d received bid from agent %d: cost %.2f", zone_id_, msg->agent_id, msg->bid_cost);
  }
}

void JobZoneLeader::timer_callback() {
  bid_timer_->cancel();
  collecting_bids_ = false;
  if (current_bids_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "ZoneLeader %d: No bids for job %d", zone_id_, current_job_.job_id);
    return;
  }
  auto best_bid = std::min_element(current_bids_.begin(), current_bids_.end(),
    [](const dirac_msgs::msg::JobBid& a, const dirac_msgs::msg::JobBid& b) {
      return a.bid_cost < b.bid_cost;
    });
  dirac_msgs::msg::Job assigned_job = current_job_;
  assigned_job.assigned_agent_id = best_bid->agent_id;
  assigned_job.status = "assigned";
  // -> Assigning jobs faster than you can say "Manhattan!"
  jobs_pub_->publish(assigned_job);
  RCLCPP_INFO(node_->get_logger(), "ZoneLeader %d assigned job %d to agent %d", zone_id_, assigned_job.job_id, best_bid->agent_id);
}

JobAgent::JobAgent(const rclcpp::Node::SharedPtr& node, int agent_id, int zone_id)
  : node_(node), agent_id_(agent_id), zone_id_(zone_id) {
  node_->get_parameter("agent_x", agent_x_);
  node_->get_parameter("agent_y", agent_y_);
  jobs_sub_ = node_->create_subscription<dirac_msgs::msg::Job>(
    "/zone_" + std::to_string(zone_id_) + "/jobs", 10,
    std::bind(&JobAgent::job_callback, this, std::placeholders::_1));
  std::string bid_topic = "/zone_" + std::to_string(zone_id_) + "/job_bids";
  bid_pub_ = node_->create_publisher<dirac_msgs::msg::JobBid>(bid_topic, 10);
}

void JobAgent::set_busy(bool busy) {
  busy_ = busy; // if agent is busy, it won't bid on new jobs
  RCLCPP_INFO(node_->get_logger(), "Agent %d is now %s", agent_id_, busy ? "busy" : "available");
}

void JobAgent::job_callback(const dirac_msgs::msg::Job::SharedPtr msg) {
  if (busy_) return;
  // Only bid if job is unassigned (assigned_agent_id == -1 or status == "unassigned")
  if (msg->assigned_agent_id != -1 || msg->status != "unassigned") return;
  // Calculate Manhattan distance as bid cost using real agent position
  double bid_cost = std::abs(agent_x_ - msg->x) + std::abs(agent_y_ - msg->y);
  dirac_msgs::msg::JobBid bid;
  bid.job_id = msg->job_id;
  bid.agent_id = agent_id_;
  bid.bid_cost = bid_cost;
  // -> Placing my bid like it's eBay at midnight!
  bid_pub_->publish(bid);
  RCLCPP_INFO(node_->get_logger(), "Agent %d bidding on job %d with cost %.2f (my pos: %.2f, %.2f)", agent_id_, msg->job_id, bid_cost, agent_x_, agent_y_);
}

} 
