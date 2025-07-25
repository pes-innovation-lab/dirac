#pragma once
#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "dirac_msgs/msg/election.hpp"

#include "dirac_msgs/msg/leader_announcement.hpp"

namespace dirac_lib {


class ElectionManager {
public:
    ElectionManager(
        int agent_id,
        int zone_id,
        double agent_x,
        double agent_y,
        rclcpp::Node::SharedPtr node);

    void startElection();
    void handleElectionMessage(int sender_id, double sender_distance, bool re_election);
    void checkElectionResult();

    bool isLeader() const;
    bool isElectionDone() const;
    int getLeaderId() const;

private:
    static constexpr double MAP_SIZE = 30.0; 
    static constexpr int ZONES_PER_ROW = 3;   

    int agent_id_;
    int zone_id_;
    double agent_x_;
    double agent_y_;
    double distance_to_center_;
    bool is_leader_ = false;
    bool election_done_ = false;

    std::unordered_map<int, double> received_distances_;
    mutable std::mutex mutex_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<dirac_msgs::msg::Election>::SharedPtr election_pub_;
    rclcpp::Subscription<dirac_msgs::msg::Election>::SharedPtr election_sub_;
    rclcpp::Publisher<dirac_msgs::msg::LeaderAnnouncement>::SharedPtr leader_announcement_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr result_timer_;
    bool stop_publishing_ = false;

    void publishElectionInfo();
    void onElectionMsg(const dirac_msgs::msg::Election::SharedPtr msg);

    void calculateZoneCenter(double& cx, double& cy) const;
    double calculateDistanceToCenter() const;
};

} 
