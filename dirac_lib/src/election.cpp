#include "dirac_lib/election.hpp"
#include "dirac_msgs/msg/leader_announcement.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dirac_lib {

ElectionManager::ElectionManager(
    int agent_id,
    int zone_id,
    double agent_x,
    double agent_y,
    rclcpp::Node::SharedPtr node)
    : agent_id_(agent_id), zone_id_(zone_id), agent_x_(agent_x), agent_y_(agent_y), node_(node)
{
    double cx, cy;
    calculateZoneCenter(cx, cy);
    RCLCPP_INFO(node_->get_logger(), "[ID %d] Zone %d center: (%.2f, %.2f)", agent_id_, zone_id_, cx, cy);
    distance_to_center_ = std::sqrt((agent_x_ - cx) * (agent_x_ - cx) + (agent_y_ - cy) * (agent_y_ - cy));

    std::string topic = "/zone_" + std::to_string(zone_id_) + "/election";
    election_pub_ = node_->create_publisher<dirac_msgs::msg::Election>(topic, 10);
    election_sub_ = node_->create_subscription<dirac_msgs::msg::Election>(
        topic, 10,
        std::bind(&ElectionManager::onElectionMsg, this, std::placeholders::_1));

    std::string leader_topic = "/zone_" + std::to_string(zone_id_) + "/leader_announcement";
    leader_announcement_pub_ = node_->create_publisher<dirac_msgs::msg::LeaderAnnouncement>(leader_topic, 10);

    publish_timer_ = node_->create_wall_timer(1s, [this]() {
        if (!stop_publishing_) this->publishElectionInfo();
    });
    result_timer_ = node_->create_wall_timer(5s, [this]() { this->checkElectionResult(); });

    RCLCPP_INFO(node_->get_logger(), "ElectionManager for ID %d in zone %d, distance to center: %.2f", agent_id_, zone_id_, distance_to_center_);
}

void ElectionManager::startElection() {
    // TODO: Implement election logic while scaling up and for reelection 
}

void ElectionManager::handleElectionMessage(int sender_id, double sender_distance, bool re_election) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (re_election) {
        received_distances_.clear();
        election_done_ = false;
        is_leader_ = false;
        RCLCPP_INFO(node_->get_logger(), "[ID %d] Re-election triggered!", agent_id_);
    } else {
        received_distances_[sender_id] = sender_distance;
        //RCLCPP_INFO(node_->get_logger(), "[%s] Received distance %.2f from %s", agent_id_.c_str(), sender_distance, sender_id.c_str());
    }
}

void ElectionManager::checkElectionResult() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (election_done_) return;
    received_distances_[agent_id_] = distance_to_center_;
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
        node_->set_parameter(rclcpp::Parameter("isLeader", is_leader_));
        node_->set_parameter(rclcpp::Parameter("z_leader", leader->first));

        dirac_msgs::msg::LeaderAnnouncement leader_msg;
        leader_msg.zone_id = zone_id_;
        leader_msg.leader_id = leader->first;
        leader_announcement_pub_->publish(leader_msg);

        RCLCPP_INFO(node_->get_logger(), "[ID %d] Election done! Leader: %d (distance: %.2f)", agent_id_, leader->first, leader->second);
        if (is_leader_) {
            RCLCPP_INFO(node_->get_logger(), "[ID %d] I am the LEADER for zone %d!", agent_id_, zone_id_);
        }
    }
}

bool ElectionManager::isLeader() const {
    return is_leader_;
}

bool ElectionManager::isElectionDone() const {
    return election_done_;
}

int ElectionManager::getLeaderId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto leader = std::min_element(
        received_distances_.begin(), received_distances_.end(),
        [](const auto& a, const auto& b) {
            if (a.second != b.second)
                return a.second < b.second;
            return a.first < a.first;
        });
    if (leader != received_distances_.end())
        return leader->first;
    return -1;
}

void ElectionManager::publishElectionInfo() {
    auto msg = dirac_msgs::msg::Election();
    msg.agent_id = agent_id_;
    msg.zone_id = zone_id_;
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
    int row = (zone_id_ - 1) / ZONES_PER_ROW;
    int col = (zone_id_ - 1) % ZONES_PER_ROW;
    double zone_size = MAP_SIZE / ZONES_PER_ROW;
    cx = col * zone_size + zone_size / 2.0;
    cy = row * zone_size + zone_size / 2.0;
}

double ElectionManager::calculateDistanceToCenter() const {
    double cx, cy;
    calculateZoneCenter(cx, cy);
    return std::sqrt((agent_x_ - cx) * (agent_x_ - cx) + (agent_y_ - cy) * (agent_y_ - cy));
}

}
