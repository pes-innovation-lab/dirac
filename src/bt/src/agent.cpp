#include <string>
#include "rclcpp/rclcpp.hpp"
#include "bt/file_io.hpp"

class Agent : public rclcpp::Node {
public:
    Agent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("agent", options) {
        this->declare_parameter<int>("agent_id", 1);
        agent_id_ = this->get_parameter("agent_id").as_int();
        auto [start, job_id, goal] = bt::FileIO::get_agent_info("src/bt/agents.csv", agent_id_);
        start_ = start;
        job_id_ = job_id;
        goal_ = goal;
        map_ = bt::FileIO::read_map_csv("src/bt/map.csv");
        ideal_path_ = bt::PathPlanner::astar_path(start_.first, start_.second, goal_.first, goal_.second, map_);
        RCLCPP_INFO(this->get_logger(), "Agent %d start: (%d,%d), job_id: %s, goal: (%d,%d)", agent_id_, start_.first, start_.second, job_id_.c_str(), goal_.first, goal_.second);
        RCLCPP_INFO(this->get_logger(), "Loaded map of size %lux%lu", map_.size(), map_.empty() ? 0 : map_[0].size());
        RCLCPP_INFO(this->get_logger(), "Ideal path length: %lu", ideal_path_.size());
    }
    

private:
    int agent_id_;
    std::pair<int, int> start_;
    std::string job_id_;
    std::pair<int, int> goal_;
    std::vector<std::vector<int>> map_;
    std::vector<std::pair<int, int>> ideal_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Agent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
