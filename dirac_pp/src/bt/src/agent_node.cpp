#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dirac_msgs/msg/agent_command.hpp"
#include "dirac_msgs/msg/job_top.hpp"
#include "std_msgs/msg/int32.hpp"
#include "bt/path_planning_handler.hpp"

class AgentNode : public rclcpp::Node {
public:
    AgentNode() : Node("agent_node") {
        // Declare parameters
        this->declare_parameter<int>("agent_id", 1);
        this->declare_parameter<int>("zone_id", 1);
        this->declare_parameter<std::string>("map_csv_path", "");
        this->declare_parameter<std::string>("agents_csv_path", "");
        this->declare_parameter<int>("tick_delay_seconds", 2);
        this->declare_parameter<bool>("enable_debug_logging", false);
        
        // Get parameters
        int agent_id = this->get_parameter("agent_id").as_int();
        int zone_id = this->get_parameter("zone_id").as_int();
        
        // Configure path planning handler
        bt::PathPlanningHandler::Config config;
        config.agent_id = agent_id;
        config.zone_id = zone_id;
        config.map_csv_path = this->get_parameter("map_csv_path").as_string();
        config.agents_csv_path = this->get_parameter("agents_csv_path").as_string();
        config.tick_delay_seconds = this->get_parameter("tick_delay_seconds").as_int();
        config.enable_debug_logging = this->get_parameter("enable_debug_logging").as_bool();
        
        // Create path planning handler
        path_handler_ = std::make_unique<bt::PathPlanningHandler>(this, config);
        
        // Set up callbacks
        path_handler_->set_command_publish_callback(
            [this](const dirac_msgs::msg::AgentCommand& cmd) {
                this->publish_command(cmd);
            }
        );
        
        path_handler_->set_log_callback(
            [this](const std::string& level, const std::string& message) {
                this->handle_log(level, message);
            }
        );
        
        // Initialize the handler
        if (!path_handler_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize path planning handler");
            return;
        }
        
        // Create publishers
        std::string command_topic = "agent_command_" + std::to_string(agent_id);
        command_pub_ = this->create_publisher<dirac_msgs::msg::AgentCommand>(command_topic, 10);
        
        // Create subscribers
        std::string job_topic = "job_top_" + std::to_string(zone_id);
        job_sub_ = this->create_subscription<dirac_msgs::msg::JobTop>(
            job_topic, 10,
            [this](dirac_msgs::msg::JobTop::SharedPtr msg) {
                path_handler_->handle_job_top_message(msg);
            }
        );
        
        zone_pop_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "zone_pop", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg) {
                path_handler_->handle_zone_pop_message(msg);
            }
        );
        
        RCLCPP_INFO(this->get_logger(), 
                   "AgentNode initialized for agent %d, subscribing to %s and zone_pop", 
                   agent_id, job_topic.c_str());
    }
    
    ~AgentNode() {
        if (path_handler_) {
            path_handler_->shutdown();
        }
    }
    
    // Status getters for external monitoring
    bool is_initialized() const {
        return path_handler_ && path_handler_->is_initialized();
    }
    
    std::pair<int, int> get_current_position() const {
        if (path_handler_) {
            return path_handler_->get_current_position();
        }
        return {-1, -1};
    }
    
    bool is_goal_reached() const {
        return path_handler_ && path_handler_->is_goal_reached();
    }

private:
    void publish_command(const dirac_msgs::msg::AgentCommand& command) {
        if (command_pub_) {
            command_pub_->publish(command);
        }
    }
    
    void handle_log(const std::string& level, const std::string& message) {
        if (level == "ERROR") {
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        } else if (level == "WARN") {
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        }
    }

    std::unique_ptr<bt::PathPlanningHandler> path_handler_;
    rclcpp::Publisher<dirac_msgs::msg::AgentCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<dirac_msgs::msg::JobTop>::SharedPtr job_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr zone_pop_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
