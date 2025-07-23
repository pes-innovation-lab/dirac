#include "dirac_navigation/publishers/publisher_manager.hpp"
#include <stdexcept>

namespace dirac_navigation
{
namespace publishers
{

std::string TopicConfig::generateTopicName(int32_t agent_id) const
{
    if (simulation_mode) {
        // Simulation mode: publish to turtle-specific topics
        return "/turtle" + std::to_string(agent_id) + "/" + base_topic;
    } else {
        // Non-simulation mode: publish to namespaced robot topics
        return "/robot" + std::to_string(agent_id) + "/" + base_topic;
    }
}

PublisherManager::PublisherManager(rclcpp::Node::SharedPtr node, const TopicConfig& config)
    : node_(node), config_(config)
{
    if (!node_) {
        throw std::invalid_argument("Node cannot be null");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Publisher Manager initialized in %s mode with base topic '%s'",
                config_.simulation_mode ? "simulation" : "real robot", config_.base_topic.c_str());
}

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PublisherManager::getPublisher(int32_t agent_id)
{
    // Check if publisher already exists for this agent
    auto it = publishers_.find(agent_id);
    if (it != publishers_.end()) {
        RCLCPP_DEBUG(node_->get_logger(), "Returning existing publisher for agent %d", agent_id);
        return it->second;
    }
    
    // Create new publisher for this agent
    return createPublisher(agent_id);
}

void PublisherManager::updateConfig(const TopicConfig& config)
{
    RCLCPP_INFO(node_->get_logger(), "Updating publisher configuration - mode: %s, base_topic: %s",
                config.simulation_mode ? "simulation" : "real robot", config.base_topic.c_str());
    
    config_ = config;
    
    // Clear existing publishers to force recreation with new configuration
    clearAll();
}

void PublisherManager::removePublisher(int32_t agent_id)
{
    auto it = publishers_.find(agent_id);
    if (it != publishers_.end()) {
        RCLCPP_INFO(node_->get_logger(), "Removing publisher for agent %d", agent_id);
        publishers_.erase(it);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Attempted to remove non-existent publisher for agent %d", agent_id);
    }
}

void PublisherManager::clearAll()
{
    if (!publishers_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Clearing all %zu publishers", publishers_.size());
        publishers_.clear();
    }
}

size_t PublisherManager::getPublisherCount() const
{
    return publishers_.size();
}

bool PublisherManager::hasPublisher(int32_t agent_id) const
{
    return publishers_.find(agent_id) != publishers_.end();
}

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr PublisherManager::createPublisher(int32_t agent_id)
{
    std::string topic_name = config_.generateTopicName(agent_id);
    
    RCLCPP_INFO(node_->get_logger(), "Creating publisher for agent %d on topic '%s'", 
                agent_id, topic_name.c_str());
    
    auto publisher = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
    
    if (!publisher) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create publisher for agent %d", agent_id);
        return nullptr;
    }
    
    publishers_[agent_id] = publisher;
    
    RCLCPP_INFO(node_->get_logger(), "Successfully created publisher for agent %d on topic '%s'", 
                agent_id, topic_name.c_str());
    
    return publisher;
}

} // namespace publishers
} // namespace dirac_navigation
