# Example usage of PathPlanningHandler

This example shows how to integrate the `PathPlanningHandler` into your own agent node.

## Basic Usage

```cpp
#include "bt/path_planning_handler.hpp"
#include "rclcpp/rclcpp.hpp"

class MyAgentNode : public rclcpp::Node {
public:
    MyAgentNode() : Node("my_agent") {
        // Configure the handler
        bt::PathPlanningHandler::Config config;
        config.agent_id = 1;
        config.zone_id = 1;
        config.tick_delay_seconds = 2;
        
        // Create the handler
        path_handler_ = std::make_unique<bt::PathPlanningHandler>(this, config);
        
        // Set up command publishing callback
        path_handler_->set_command_publish_callback(
            [this](const dirac_msgs::msg::AgentCommand& cmd) {
                // Handle movement commands from the path planner
                this->execute_movement_command(cmd);
            }
        );
        
        // Initialize
        path_handler_->initialize();
        
        // Create subscribers for job and zone data
        job_sub_ = this->create_subscription<dirac_msgs::msg::JobTop>(
            "job_top_1", 10,
            [this](dirac_msgs::msg::JobTop::SharedPtr msg) {
                path_handler_->handle_job_top_message(msg);
            }
        );
        
        zone_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "zone_pop", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg) {
                path_handler_->handle_zone_pop_message(msg);
            }
        );
    }
    
private:
    void execute_movement_command(const dirac_msgs::msg::AgentCommand& cmd) {
        // Implement your movement logic here
        RCLCPP_INFO(this->get_logger(), "Executing move direction: %d", cmd.direction);
        
        // Example: Send to robot hardware, simulation, etc.
        // robot_interface_->move(cmd.direction);
    }
    
    std::unique_ptr<bt::PathPlanningHandler> path_handler_;
    rclcpp::Subscription<dirac_msgs::msg::JobTop>::SharedPtr job_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr zone_sub_;
};
```

## Configuration Options

The `PathPlanningHandler::Config` struct provides several configuration options:

- `agent_id`: Unique identifier for this agent
- `zone_id`: Zone identifier for topic subscription
- `map_csv_path`: Path to map file (optional, defaults to package share)
- `agents_csv_path`: Path to agents file (optional, defaults to package share)
- `tick_delay_seconds`: Delay between ticks for observation
- `enable_debug_logging`: Enable additional debug output

## Status Monitoring

You can monitor the handler's status:

```cpp
// Check if handler is fully initialized
bool ready = path_handler_->is_initialized();

// Check if job data has been received
bool has_job = path_handler_->has_received_job_data();

// Get current position
auto pos = path_handler_->get_current_position();

// Check if goal is reached
bool done = path_handler_->is_goal_reached();
```

## Integration with Existing Projects

To use this handler in your existing project:

1. Add `bt` as a dependency in your `package.xml`:
```xml
<depend>bt</depend>
```

2. Add it to your `CMakeLists.txt`:
```cmake
find_package(bt REQUIRED)
target_link_libraries(your_target path_planning_handler)
```

3. Include the header and use as shown above.

## Topics

The handler expects these topics:
- `job_top_${zone_id}`: Publishes `dirac_msgs::msg::JobTop` messages
- `zone_pop`: Publishes `std_msgs::msg::Int32` messages

The handler will call your command callback when movement is needed.
