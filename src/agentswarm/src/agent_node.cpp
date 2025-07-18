#include <string>
#include <cmath>     
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#include <map>
#include <chrono>
#include <set>
#include "rclcpp/rclcpp.hpp"
#include "agentswarm/msg/agent_distance.hpp"
#include "agentswarm/msg/heartbeat.hpp"
#include "agentswarm/msg/job_assignment.hpp"
#include "agentswarm/msg/job_biīd.hpp"

const double MAP_SIZE = 30.0;
const int ZONES_PER_ROW = 3;

class AgentNode : public rclcpp::Node {
public:
    AgentNode();

private:
    std::set<int> current_jobs_; // > tracks jobs this agent is working on, usually 1
    rclcpp::Subscription<agentswarm::msg::AgentDistance>::SharedPtr election_;
    rclcpp::Publisher<agentswarm::msg::AgentDistance>::SharedPtr election_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr election_timer_;
    rclcpp::Publisher<agentswarm::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    rclcpp::Subscription<agentswarm::msg::Heartbeat>::SharedPtr heartbeat_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;
    std::map<int, rclcpp::Time> last_heartbeat_;
    bool is_leader_ = false; 
    int agent_id_;
    int zone_id_;
    double pos_x_, pos_y_;
    double center_x_, center_y_;
    double distance_to_center_;
    std::map<int, double> agent_distances_; // > stores distance to center for each agent
    bool election_done_ = false;
    std::set<int> received_agents_; // > agents that have sent election info
    std::set<int> zone_agent_ids_; // > all agent ids in this zone
    rclcpp::Publisher<agentswarm::msg::JobAssignment>::SharedPtr incoming_jobs_pub_;
    rclcpp::Subscription<agentswarm::msg::JobAssignment>::SharedPtr incoming_jobs_sub_;
    std::map<int, rclcpp::Publisher<agentswarm::msg::JobAssignment>::SharedPtr> zone_jobs_pubs_;
    rclcpp::Subscription<agentswarm::msg::JobAssignment>::SharedPtr zone_jobs_sub_;
    rclcpp::Publisher<agentswarm::msg::JobAssignment>::SharedPtr jobs_pub_;
    rclcpp::Subscription<agentswarm::msg::JobAssignment>::SharedPtr jobs_sub_;
    rclcpp::Publisher<agentswarm::msg::JobBid>::SharedPtr job_bid_pub_;
    rclcpp::Subscription<agentswarm::msg::JobBid>::SharedPtr job_bid_sub_;
    int job_id_counter_ = 0;
    std::map<int, agentswarm::msg::JobAssignment> active_jobs_; // > jobs being managed by this agent
    std::map<int, std::map<int, double>> job_bids_; // > job_id -> (agent_id -> cost)
    std::map<int, rclcpp::TimerBase::SharedPtr> job_timers_;

    void get_zone_center(int zone_id, double &cx, double &cy);
    void publish_own_info();
    void election_function(const agentswarm::msg::AgentDistance::SharedPtr msg);
    void run_election();
    int calculate_zone(double x, double y);
    void route_job_to_zone(const agentswarm::msg::JobAssignment::SharedPtr job);
    void process_zone_job(const agentswarm::msg::JobAssignment::SharedPtr job);
    void bid_on_job(const agentswarm::msg::JobAssignment::SharedPtr job);
    void process_job_bid(const agentswarm::msg::JobBid::SharedPtr bid);
    void assign_job_to_best_bidder(int job_id);
    void execute_job(const agentswarm::msg::JobAssignment::SharedPtr job);
    void complete_job(int job_id);
};

AgentNode::AgentNode() : Node("agent_node") {
    // > get params for agent position and id
    this->declare_parameter<double>("pos_x", 0.0);
    this->declare_parameter<double>("pos_y", 0.0);
    this->declare_parameter<int>("agent_id", 0);
    double pos_x = this->get_parameter("pos_x").as_double();
    double pos_y = this->get_parameter("pos_y").as_double();
    agent_id_ = this->get_parameter("agent_id").as_int();
    int zone = calculate_zone(pos_x, pos_y); // > figure out which zone this agent is in
    zone_id_ = zone;
    pos_x_ = pos_x;
    pos_y_ = pos_y;

    // > setup election pub/sub for this zone
    std::string z_topic = "/zone_" + std::to_string(zone) + "/election";
    election_ = this->create_subscription<agentswarm::msg::AgentDistance>(
        z_topic, 10, std::bind(&AgentNode::election_function, this, std::placeholders::_1));
    election_pub_ = this->create_publisher<agentswarm::msg::AgentDistance>(z_topic, 10);

    // > if this agent is leader, send heartbeat
    if (is_leader_) {
        heartbeat_pub_ = this->create_publisher<agentswarm::msg::Heartbeat>("/heartbeat_radio", 10);
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // > send heartbeat every 0.5s or seconds.. need to decide
            [this]() {
                auto msg = std::make_unique<agentswarm::msg::Heartbeat>();
                msg->agent_id = agent_id_;
                msg->zone_id = zone_id_;
                heartbeat_pub_->publish(*msg);
            }
        );
    }
    // > agent 1 acts as "super leader" for job routing and heartbeat monitoring
    if (agent_id_ == 1) {
        heartbeat_sub_ = this->create_subscription<agentswarm::msg::Heartbeat>(
            "/heartbeat_radio", 10,
            [this](const agentswarm::msg::Heartbeat::SharedPtr msg) {
                last_heartbeat_[msg->zone_id] = this->now();
            }
        );
        heartbeat_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                rclcpp::Time now = this->now();
                for (const auto& kv : last_heartbeat_) {
                    int zone = kv.first;
                    rclcpp::Duration elapsed = now - kv.second;
                    if (elapsed.seconds() > 2.0) {
                        RCLCPP_WARN(this->get_logger(), "No heartbeat from zone %d", zone);
                    }
                }
            }
        );
        incoming_jobs_pub_ = this->create_publisher<agentswarm::msg::JobAssignment>("/incoming_jobs", 10);
        incoming_jobs_sub_ = this->create_subscription<agentswarm::msg::JobAssignment>(
            "/incoming_jobs", 10,
            [this](const agentswarm::msg::JobAssignment::SharedPtr msg) {
                route_job_to_zone(msg); // > super leader sends job to correct zone
            }
        );
    }
    get_zone_center(zone_id_, center_x_, center_y_); // > find center of this agent's zone
    distance_to_center_ = std::sqrt(std::pow(pos_x_ - center_x_, 2) + std::pow(pos_y_ - center_y_, 2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&AgentNode::publish_own_info, this)
    );
    election_timer_ = this->create_wall_timer(
        std::chrono::seconds(15),
        std::bind(&AgentNode::run_election, this)
    );
    // > subscribe to jobs for this zone
    std::string zone_jobs_topic = "/zone_" + std::to_string(zone_id_) + "/jobs";
    zone_jobs_sub_ = this->create_subscription<agentswarm::msg::JobAssignment>(
        zone_jobs_topic, 10,
        [this](const agentswarm::msg::JobAssignment::SharedPtr msg) {
            if (is_leader_) process_zone_job(msg); // > only leader handles jobs
        }
    );
    jobs_pub_ = this->create_publisher<agentswarm::msg::JobAssignment>("/jobs", 10);
    jobs_sub_ = this->create_subscription<agentswarm::msg::JobAssignment>(
        "/jobs", 10,
        [this](const agentswarm::msg::JobAssignment::SharedPtr msg) {
            if (!is_leader_ && msg->assigned_agent_id == -1 && !msg->completed) {
                bid_on_job(msg); // > regular agents bid for unassigned jobs
            } else if (msg->assigned_agent_id == agent_id_ && !msg->completed) {
                execute_job(msg); // > if job assigned to me, start it
            }
        }
    );
    std::string zone_job_bids_topic = "/zone_" + std::to_string(zone_id_) + "/job_bids";
    job_bid_pub_ = this->create_publisher<agentswarm::msg::JobBid>(zone_job_bids_topic, 10);
    job_bid_sub_ = this->create_subscription<agentswarm::msg::JobBid>(
        zone_job_bids_topic, 10,
        [this](const agentswarm::msg::JobBid::SharedPtr msg) {
            if (is_leader_) process_job_bid(msg); // > leader collects bids
        }
    );
}

void AgentNode::get_zone_center(int zone_id, double &cx, double &cy) {
    // > calculate center of a zone based on id
    int row = (zone_id - 1) / ZONES_PER_ROW;
    int col = (zone_id - 1) % ZONES_PER_ROW;
    double zone_size = MAP_SIZE / ZONES_PER_ROW;
    cx = col * zone_size + zone_size / 2.0;
    cy = row * zone_size + zone_size / 2.0;
}

void AgentNode::publish_own_info() {
    // > send my distance to center for election
    if (election_done_) return;
    auto msg = std::make_unique<agentswarm::msg::AgentDistance>();
    msg->agent_id = agent_id_;
    msg->distance = distance_to_center_;
    msg->re_election = false;
    election_pub_->publish(*msg);
}

void AgentNode::election_function(const agentswarm::msg::AgentDistance::SharedPtr msg) {
    // > handle election messages, reset if re-election triggered
    if (msg->re_election) {
        RCLCPP_WARN(this->get_logger(), "[Election] Agent %d (zone %d) received re-election trigger. State reset.", agent_id_, zone_id_);
    }
    if (msg->re_election) {
        is_leader_ = false;
        agent_distances_.clear();
        received_agents_.clear();
        zone_agent_ids_.clear();
        election_done_ = false;
        return;
    }
    if (election_done_) return;
    agent_distances_[msg->agent_id] = msg->distance;
    received_agents_.insert(msg->agent_id);
    zone_agent_ids_.insert(msg->agent_id);
}

void AgentNode::run_election() {
    // > pick leader: agent closest to center (break ties by id)
    if (election_done_) return;
    election_done_ = true;
    agent_distances_[agent_id_] = distance_to_center_;
    int leader_id = agent_id_;
    double min_dist = distance_to_center_;
    for (const auto& kv : agent_distances_) {
        if (kv.second < min_dist || (kv.second == min_dist && kv.first < leader_id)) {
            min_dist = kv.second;
            leader_id = kv.first;
        }
    }
    is_leader_ = (agent_id_ == leader_id);
    if (is_leader_) {
        RCLCPP_INFO(this->get_logger(), "[Election] Zone %d leader selected: Agent %d", zone_id_, agent_id_);
    }
}

int AgentNode::calculate_zone(double x, double y) {
    // > map (x, y) to zone id, returns -1 if out of bounds
    if (x < 0 || x > MAP_SIZE || y < 0 || y > MAP_SIZE) return -1; // > out of bounds
    double zone_size = MAP_SIZE / ZONES_PER_ROW;
    int col_index = static_cast<int>(std::floor(x / zone_size));
    int row_index = static_cast<int>(std::floor(y / zone_size));
    col_index = std::min(col_index, ZONES_PER_ROW - 1);
    row_index = std::min(row_index, ZONES_PER_ROW - 1);
    return row_index * ZONES_PER_ROW + col_index + 1;
}

void AgentNode::route_job_to_zone(const agentswarm::msg::JobAssignment::SharedPtr job) {
    // > super leader sends job to correct zone topic
    int job_zone = calculate_zone(job->x, job->y);
    if (job_zone < 1 || job_zone > ZONES_PER_ROW * ZONES_PER_ROW) {
        RCLCPP_ERROR(this->get_logger(), "[Super Leader] Job %d coordinates (%.2f, %.2f) are outside map bounds!", job->job_id, job->x, job->y);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[SUPER LEADER] Routing job %d to zone %d", job->job_id, job_zone);
    if (zone_jobs_pubs_.find(job_zone) == zone_jobs_pubs_.end()) {
        std::string topic = "/zone_" + std::to_string(job_zone) + "/jobs";
        zone_jobs_pubs_[job_zone] = this->create_publisher<agentswarm::msg::JobAssignment>(topic, 10);
    }
    agentswarm::msg::JobAssignment routed_job = *job; // > create a new JobAssignment message to route to the correct zone
    routed_job.zone_id = job_zone;
    zone_jobs_pubs_[job_zone]->publish(routed_job);
}

void AgentNode::process_zone_job(const agentswarm::msg::JobAssignment::SharedPtr job) {
    // > leader receives job for this zone, starts bidding process
    if (!is_leader_ || job->zone_id != zone_id_) return;
    RCLCPP_INFO(this->get_logger(), "[ZONE LEADER %d] Received job %d for zone %d", zone_id_, job->job_id, job->zone_id);
    active_jobs_[job->job_id] = *job;
    job_bids_[job->job_id].clear();
    jobs_pub_->publish(*job); // > broadcast job to all agents in zone
    job_timers_[job->job_id] = this->create_wall_timer(
        std::chrono::seconds(2),
        [this, job_id=job->job_id]() {
            assign_job_to_best_bidder(job_id); // > after 2s, pick winner
            job_timers_[job_id]->cancel();
            job_timers_.erase(job_id);
        }
    );
}

void AgentNode::bid_on_job(const agentswarm::msg::JobAssignment::SharedPtr job) {
    // > agent bids for job if not busy
    if (job->zone_id != zone_id_) return;
    if (!current_jobs_.empty()) {
        RCLCPP_INFO(this->get_logger(), "[Agent %d] Busy with other job(s), not bidding on job %d", agent_id_, job->job_id);
        return;
    }
    double cost = std::abs(pos_x_ - job->x) + std::abs(pos_y_ - job->y); // > manhattan distance
    RCLCPP_INFO(this->get_logger(), "[Agent %d] Bidding on job %d with cost %.2f", agent_id_, job->job_id, cost);
    auto bid = std::make_unique<agentswarm::msg::JobBid>();
    bid->job_id = job->job_id;
    bid->agent_id = agent_id_;
    bid->cost = cost;
    job_bid_pub_->publish(*bid);
}

void AgentNode::process_job_bid(const agentswarm::msg::JobBid::SharedPtr bid) {
    // > leader collects all bids for a job
    job_bids_[bid->job_id][bid->agent_id] = bid->cost;
}

void AgentNode::assign_job_to_best_bidder(int job_id) {
    // > pick agent with lowest cost, assign job
    if (active_jobs_.find(job_id) != active_jobs_.end() && !job_bids_[job_id].empty()) {
        int best_agent = -1;
        double best_cost = std::numeric_limits<double>::max();
        for (const auto& bid : job_bids_[job_id]) {
            if (bid.second < best_cost) {
                best_cost = bid.second;
                best_agent = bid.first;
            }
        }
        RCLCPP_INFO(this->get_logger(), "[ZONE LEADER %d] Job %d assigned to AGENT %d (cost %.2f)", zone_id_, job_id, best_agent, best_cost);
    }
    if (active_jobs_.find(job_id) == active_jobs_.end()) {
        RCLCPP_WARN(this->get_logger(), "[Zone Leader %d] No active job with ID %d found", zone_id_, job_id);
        return;
    }
    const auto& bids = job_bids_[job_id];
    if (bids.empty()) {
        RCLCPP_WARN(this->get_logger(), "[Zone Leader %d] No bids received for job %d", zone_id_, job_id);
        return;
    }
    int best_agent = -1;
    double best_cost = std::numeric_limits<double>::max();
    for (const auto& bid : bids) {
        if (bid.second < best_cost) {
            best_cost = bid.second;
            best_agent = bid.first;
        }
    }
    auto& job = active_jobs_[job_id];
    job.assigned_agent_id = best_agent;
    RCLCPP_INFO(this->get_logger(), "[ZONE LEADER %d] Job %d assigned to AGENT %d (cost %.2f)", zone_id_, job_id, best_agent, best_cost);
    jobs_pub_->publish(job);
}

void AgentNode::execute_job(const agentswarm::msg::JobAssignment::SharedPtr job) {
    // > agent starts working on assigned job
    current_jobs_.insert(job->job_id); // -> now busy
    job_timers_[job->job_id] = this->create_wall_timer(
        std::chrono::seconds(3),
        [this, job_id=job->job_id]() {
            complete_job(job_id); // > after 3s, mark job done
            job_timers_[job_id]->cancel();
            job_timers_.erase(job_id);
        }
    );
}

void AgentNode::complete_job(int job_id) {
    // > agent marks job as complete, notifies others for now its not   
    auto it = active_jobs_.find(job_id);
    agentswarm::msg::JobAssignment completed_job;
    if (it != active_jobs_.end()) {
        completed_job = it->second;
    } else {
        completed_job.job_id = job_id;
        completed_job.assigned_agent_id = agent_id_;
    }
    completed_job.completed = true;
    RCLCPP_INFO(this->get_logger(), "[Agent %d] Completed job %d", agent_id_, job_id);
    jobs_pub_->publish(completed_job);
    current_jobs_.erase(job_id); // -> now free
}

int main(int argc, char *argv[]) {
    // > start ros2 node for agent
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
