#include <rclcpp/rclcpp.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include "dirac_navigation/navigation_controller.hpp"
#include <cstdlib>

class DiscreteNavigationControllerNode : public rclcpp::Node
{
public:
    DiscreteNavigationControllerNode() : Node("discrete_navigation_controller")
    {
        // Get agent ID from environment first
        get_agent_id();
        
        // Setup topics with agent namespace
        setup_topics();

        RCLCPP_INFO(this->get_logger(), "Discrete Navigation Controller Node started for agent %d", agent_id_);
        RCLCPP_INFO(this->get_logger(), "Command mapping: 1=forward, 2=backward, 3=left, 4=right");
    }

    void initialize_navigation()
    {
        // Initialize the navigation library after the node is fully constructed
        navigation_ = std::make_shared<dirac_navigation::NavigationController>(shared_from_this());
        
        // Initialize from ROS parameters (this will handle all configuration)
        if (!navigation_->initializeFromParameters(shared_from_this())) {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize from parameters, using defaults");
        }
        
        RCLCPP_INFO(this->get_logger(), "Navigation controller initialized for agent %d", agent_id_);
    }

private:
    void get_agent_id()
    {
        // Get agent ID from environment
        agent_id_ = 1; // default
        const char* agent_id_env = std::getenv("AGENT_ID");
        if (agent_id_env) {
            try {
                agent_id_ = std::stoi(agent_id_env);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Invalid AGENT_ID environment variable, using default: 1");
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Agent ID: %d", agent_id_);
    }

    void setup_topics()
    {
        // Get simulation mode from configuration parameter (still needed for topic naming)
        this->declare_parameter("simulation_mode", true);
        bool is_simulation = this->get_parameter("simulation_mode").as_bool();
        
        // Create subscription to agent commands with robot namespace
        std::string commands_topic = "/robot" + std::to_string(agent_id_) + "/agent_commands";
        
        command_subscription_ = this->create_subscription<dirac_msgs::msg::AgentCommand>(
            commands_topic, 10,
            std::bind(&DiscreteNavigationControllerNode::command_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Listening for agent commands on '%s' topic", commands_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Running in %s mode", is_simulation ? "simulation" : "real robot");
    }

    void command_callback(const dirac_msgs::msg::AgentCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: direction=%d for agent %d", msg->direction, agent_id_);
        
        // Initialize navigation if not already done
        if (!navigation_) {
            initialize_navigation();
        }
        
        // Use the navigation library to execute the command for this agent
        navigation_->execute_command(agent_id_, msg->direction);
    }

    // Navigation library instance
    std::shared_ptr<dirac_navigation::NavigationController> navigation_;
    
    // Subscription to agent commands
    rclcpp::Subscription<dirac_msgs::msg::AgentCommand>::SharedPtr command_subscription_;
    
    // Agent ID for this controller instance
    int32_t agent_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiscreteNavigationControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
