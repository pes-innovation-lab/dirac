#include "dirac_lib/election.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dirac_lib {

ElectionManager::ElectionManager(
    const std::string& agent_id,
    const std::string& zone_id,
    double agent_x,
    double agent_y,
    rclcpp::Node::SharedPtr node)
    : agent_id_(agent_id), zone_id_(zone_id), agent_x_(agent_x), agent_y_(agent_y), node_(node)
{
    double cx, cy;
    calculateZoneCenter(cx, cy);
    RCLCPP_INFO(node_->get_logger(), "[%s] Zone %s center: (%.2f, %.2f)", agent_id_.c_str(), zone_id_.c_str(), cx, cy);
    distance_to_center_ = std::sqrt((agent_x_ - cx) * (agent_x_ - cx) + (agent_y_ - cy) * (agent_y_ - cy));

    std::string topic = "/zone_" + zone_id_ + "/election";
    election_pub_ = node_->create_publisher<dirac_msgs::msg::Election>(topic, 10);
    election_sub_ = node_->create_subscription<dirac_msgs::msg::Election>(
        topic, 10,
        std::bind(&ElectionManager::onElectionMsg, this, std::placeholders::_1));

    // Publish election info every second until election is done
    publish_timer_ = node_->create_wall_timer(1s, [this]() {
        if (!stop_publishing_) this->publishElectionInfo();
    });
    // Check election result after 15s
    result_timer_ = node_->create_wall_timer(5s, [this]() { this->checkElectionResult(); });

    RCLCPP_INFO(node_->get_logger(), "ElectionManager for %s in zone %s, distance to center: %.2f", agent_id_.c_str(), zone_id_.c_str(), distance_to_center_);
}

void ElectionManager::startElection() {
    // Not used in this simple demo, handled by timer
}

void ElectionManager::handleElectionMessage(const std::string& sender_id, double sender_distance, bool re_election) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (re_election) {
        received_distances_.clear();
        election_done_ = false;
        is_leader_ = false;
        RCLCPP_INFO(node_->get_logger(), "[%s] Re-election triggered!", agent_id_.c_str());
    } else {
        received_distances_[sender_id] = sender_distance;
        //RCLCPP_INFO(node_->get_logger(), "[%s] Received distance %.2f from %s", agent_id_.c_str(), sender_distance, sender_id.c_str());
    }
}

void ElectionManager::checkElectionResult() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (election_done_) return;
    // Include self in the map
    received_distances_[agent_id_] = distance_to_center_;
    // Find the agent with the smallest distance (break ties by agent_id)
    auto leader = std::min_element(
        received_distances_.begin(), received_distances_.end(),
        [](const auto& a, const auto& b) {
            if (a.second != b.second)
                return a.second < b.second;
            return a.first < b.first;
        });
    if (leader != received_distances_.end()) {
        is_leader_ = (leader->first == agent_id_);
        election_done_ = true;
        stop_publishing_ = true;
        if (publish_timer_) {
            publish_timer_->cancel();
        }
        // Set parameters on the node
        node_->set_parameter(rclcpp::Parameter("isLeader", is_leader_));
        node_->set_parameter(rclcpp::Parameter("z_leader", leader->first));
        RCLCPP_INFO(node_->get_logger(), "[%s] Election done! Leader: %s (distance: %.2f)", agent_id_.c_str(), leader->first.c_str(), leader->second);
        if (is_leader_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] I am the LEADER for zone %s!", agent_id_.c_str(), zone_id_.c_str());
        }
    }
}

bool ElectionManager::isLeader() const {
    return is_leader_;
}

bool ElectionManager::isElectionDone() const {
    return election_done_;
}

std::string ElectionManager::getLeaderId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    // Find the leader again for reporting
    auto leader = std::min_element(
        received_distances_.begin(), received_distances_.end(),
        [](const auto& a, const auto& b) {
            if (a.second != b.second)
                return a.second < b.second;
            return a.first < a.first;
        });
    if (leader != received_distances_.end())
        return leader->first;
    return "";
}

void ElectionManager::publishElectionInfo() {
    auto msg = dirac_msgs::msg::Election();
    msg.agent_id = agent_id_;
    msg.zone_id = std::stoi(zone_id_);
    msg.distance_to_center = distance_to_center_;
    msg.re_election = false;
    election_pub_->publish(msg);
    //RCLCPP_INFO(node_->get_logger(), "[%s] Published election info: distance %.2f", agent_id_.c_str(), distance_to_center_);
}

void ElectionManager::onElectionMsg(const dirac_msgs::msg::Election::SharedPtr msg) {
    if (msg->agent_id == agent_id_) return; // Ignore own message
    handleElectionMessage(msg->agent_id, msg->distance_to_center, msg->re_election);
}

void ElectionManager::calculateZoneCenter(double& cx, double& cy) const {
    int zone_id_int = std::stoi(zone_id_);
    int row = (zone_id_int - 1) / ZONES_PER_ROW;
    int col = (zone_id_int - 1) % ZONES_PER_ROW;
    double zone_size = MAP_SIZE / ZONES_PER_ROW;
    cx = col * zone_size + zone_size / 2.0;
    cy = row * zone_size + zone_size / 2.0;
}

double ElectionManager::calculateDistanceToCenter() const {
    double cx, cy;
    calculateZoneCenter(cx, cy);
    return std::sqrt((agent_x_ - cx) * (agent_x_ - cx) + (agent_y_ - cy) * (agent_y_ - cy));
}

} // namespace dirac_lib
