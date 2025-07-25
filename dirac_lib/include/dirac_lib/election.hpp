#pragma once


#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "dirac_msgs/msg/election.hpp"

namespace dirac_lib {


class ElectionManager {
public:
    ElectionManager(
        const std::string& agent_id,
        const std::string& zone_id,
        double agent_x,
        double agent_y,
        rclcpp::Node::SharedPtr node);

    void startElection();
    void handleElectionMessage(const std::string& sender_id, double sender_distance, bool re_election);
    void checkElectionResult();

    bool isLeader() const;
    bool isElectionDone() const;
    std::string getLeaderId() const;

private:
    // Constants for zone calculation
    static constexpr double MAP_SIZE = 30.0; // Example value, can be parameterized
    static constexpr int ZONES_PER_ROW = 3;   // Example value, can be parameterized

    std::string agent_id_;
    std::string zone_id_;
    double agent_x_;
    double agent_y_;
    double distance_to_center_;
    bool is_leader_ = false;
    bool election_done_ = false;

    std::unordered_map<std::string, double> received_distances_;
    mutable std::mutex mutex_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<dirac_msgs::msg::Election>::SharedPtr election_pub_;
    rclcpp::Subscription<dirac_msgs::msg::Election>::SharedPtr election_sub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr result_timer_;
    bool stop_publishing_ = false;

    void publishElectionInfo();
    void onElectionMsg(const dirac_msgs::msg::Election::SharedPtr msg);

    // Helper to calculate zone center and distance
    void calculateZoneCenter(double& cx, double& cy) const;
    double calculateDistanceToCenter() const;
};

} // namespace dirac_lib
