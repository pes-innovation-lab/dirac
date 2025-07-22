#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dirac_msgs/msg/agent_command.hpp>
#include <map>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        // Create subscription to agent commands
        command_subscription_ = this->create_subscription<dirac_msgs::msg::AgentCommand>(
            "agent_commands", 10,
            std::bind(&TurtleController::command_callback, this, std::placeholders::_1));

        // Movement parameters
        linear_speed_ = 2.0;   // Linear velocity
        angular_speed_ = 1.57; // Angular velocity (π/2 rad/s for 90° turns)
        move_distance_ = 1.0;  // Distance to move forward/backward (1 unit)
        turn_angle_ = 1.57;    // 90 degrees in radians (π/2)

        RCLCPP_INFO(this->get_logger(), "Turtle Controller started");
        RCLCPP_INFO(this->get_logger(), "Listening for agent commands on 'agent_commands' topic");
        RCLCPP_INFO(this->get_logger(), "Command mapping: 1=forward, 2=backward, 3=left, 4=right");
    }

private:
    void command_callback(const dirac_msgs::msg::AgentCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command for agent %d: direction %d", 
                    msg->agent_id, msg->direction);

        // Get or create publisher for this agent
        auto publisher = get_or_create_publisher(msg->agent_id);
        
        // Execute movement sequence based on direction
        switch(msg->direction) {
            case 1: // Forward
                RCLCPP_INFO(this->get_logger(), "Agent %d: Moving forward 1 unit", msg->agent_id);
                move_forward(publisher, msg->agent_id);
                break;
                
            case 2: // Backward
                RCLCPP_INFO(this->get_logger(), "Agent %d: Moving backward 1 unit", msg->agent_id);
                move_backward(publisher, msg->agent_id);
                break;
                
            case 3: // Left (turn left, move forward, turn right)
                RCLCPP_INFO(this->get_logger(), "Agent %d: Moving left (turn-move-turn sequence)", msg->agent_id);
                move_left(publisher, msg->agent_id);
                break;
                
            case 4: // Right (turn right, move forward, turn left)
                RCLCPP_INFO(this->get_logger(), "Agent %d: Moving right (turn-move-turn sequence)", msg->agent_id);
                move_right(publisher, msg->agent_id);
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "Agent %d: Invalid direction %d. Valid directions: 1=forward, 2=backward, 3=left, 4=right", 
                           msg->agent_id, msg->direction);
                return;
        }
    }

    void move_forward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_speed_;
        twist_msg.angular.z = 0.0;
        
        publisher->publish(twist_msg);
        
        // Calculate duration to move exactly 1 unit
        auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
        
        auto timer = this->create_wall_timer(duration, [this, publisher, agent_id]() {
            stop_turtle(publisher, agent_id);
        });
        
        timers_[agent_id] = timer;
    }

    void move_backward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = -linear_speed_;
        twist_msg.angular.z = 0.0;
        
        publisher->publish(twist_msg);
        
        // Calculate duration to move exactly 1 unit
        auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
        
        auto timer = this->create_wall_timer(duration, [this, publisher, agent_id]() {
            stop_turtle(publisher, agent_id);
        });
        
        timers_[agent_id] = timer;
    }

    void move_left(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        // Step 1: Turn left 90 degrees
        turn_turtle(publisher, angular_speed_, [this, publisher, agent_id]() {
            // Step 2: Move forward 1 unit
            move_forward_then_turn_right(publisher, agent_id);
        });
    }

    void move_right(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        // Step 1: Turn right 90 degrees
        turn_turtle(publisher, -angular_speed_, [this, publisher, agent_id]() {
            // Step 2: Move forward 1 unit
            move_forward_then_turn_left(publisher, agent_id);
        });
    }

    void move_forward_then_turn_right(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_speed_;
        twist_msg.angular.z = 0.0;
        
        publisher->publish(twist_msg);
        
        auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
        
        auto timer = this->create_wall_timer(duration, [this, publisher, agent_id]() {
            // Step 3: Turn right 90 degrees to reset orientation
            turn_turtle(publisher, -angular_speed_, [this, publisher, agent_id]() {
                stop_turtle(publisher, agent_id);
            });
        });
        
        timers_[agent_id] = timer;
    }

    void move_forward_then_turn_left(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_speed_;
        twist_msg.angular.z = 0.0;
        
        publisher->publish(twist_msg);
        
        auto duration = std::chrono::milliseconds(static_cast<int>((move_distance_ / linear_speed_) * 1000));
        
        auto timer = this->create_wall_timer(duration, [this, publisher, agent_id]() {
            // Step 3: Turn left 90 degrees to reset orientation
            turn_turtle(publisher, angular_speed_, [this, publisher, agent_id]() {
                stop_turtle(publisher, agent_id);
            });
        });
        
        timers_[agent_id] = timer;
    }

    void turn_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, 
                     double angular_vel, std::function<void()> callback)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = angular_vel;
        
        publisher->publish(twist_msg);
        
        // Calculate duration to turn exactly 90 degrees
        auto duration = std::chrono::milliseconds(static_cast<int>((turn_angle_ / std::abs(angular_vel)) * 1000));
        
        auto timer = this->create_wall_timer(duration, callback);
        // Note: We don't store this timer in the map since it's temporary for the sequence
    }

    void stop_turtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, int32_t agent_id)
    {
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        publisher->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Agent %d: Movement complete", agent_id);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 
    get_or_create_publisher(int32_t agent_id)
    {
        // Check if publisher already exists for this agent
        auto it = publishers_.find(agent_id);
        if (it != publishers_.end()) {
            return it->second;
        }
        
        // Create new publisher for this agent
        std::string topic_name = "/turtle" + std::to_string(agent_id) + "/cmd_vel";
        auto publisher = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
        
        publishers_[agent_id] = publisher;
        
        RCLCPP_INFO(this->get_logger(), "Created publisher for agent %d on topic '%s'", 
                    agent_id, topic_name.c_str());
        
        return publisher;
    }

    // Subscription to agent commands
    rclcpp::Subscription<dirac_msgs::msg::AgentCommand>::SharedPtr command_subscription_;
    
    // Map of agent_id to their respective cmd_vel publishers
    std::map<int32_t, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    
    // Map of agent_id to their timers (to keep timers alive)
    std::map<int32_t, rclcpp::TimerBase::SharedPtr> timers_;
    
    // Movement parameters
    double linear_speed_;
    double angular_speed_;
    double move_distance_;
    double turn_angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}
