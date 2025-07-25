#ifndef DIRAC_NAVIGATION_PUBLISHERS_PUBLISHER_MANAGER_HPP_
#define DIRAC_NAVIGATION_PUBLISHERS_PUBLISHER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <string>
#include <memory>

namespace dirac_navigation
{
namespace publishers
{

/**
 * @brief Configuration for topic naming strategy
 */
struct TopicConfig
{
    std::string base_topic = "cmd_vel";
    bool simulation_mode = true;
    
    /**
     * @brief Generate topic name for an agent
     * @param agent_id Agent identifier
     * @return Full topic name
     */
    std::string generateTopicName(int32_t agent_id) const;
};

/**
 * @brief Manages publishers for multiple agents
 */
class PublisherManager
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for creating publishers
     * @param config Topic configuration
     */
    explicit PublisherManager(rclcpp::Node::SharedPtr node, const TopicConfig& config = TopicConfig{});
    
    /**
     * @brief Get or create publisher for an agent
     * @param agent_id Agent identifier
     * @return Publisher for the agent
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr getPublisher(int32_t agent_id);
    
    /**
     * @brief Update topic configuration
     * @param config New topic configuration
     * @note This will clear existing publishers to force recreation with new topics
     */
    void updateConfig(const TopicConfig& config);
    
    /**
     * @brief Remove publisher for an agent
     * @param agent_id Agent identifier
     */
    void removePublisher(int32_t agent_id);
    
    /**
     * @brief Clear all publishers
     */
    void clearAll();
    
    /**
     * @brief Get number of active publishers
     * @return Number of publishers
     */
    size_t getPublisherCount() const;
    
    /**
     * @brief Check if publisher exists for an agent
     * @param agent_id Agent identifier
     * @return True if publisher exists
     */
    bool hasPublisher(int32_t agent_id) const;

private:
    rclcpp::Node::SharedPtr node_;
    TopicConfig config_;
    std::map<int32_t, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    
    /**
     * @brief Create a new publisher for an agent
     * @param agent_id Agent identifier
     * @return New publisher
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr createPublisher(int32_t agent_id);
};

} // namespace publishers
} // namespace dirac_navigation

#endif // DIRAC_NAVIGATION_PUBLISHERS_PUBLISHER_MANAGER_HPP_
