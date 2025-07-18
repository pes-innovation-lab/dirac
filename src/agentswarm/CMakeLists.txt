cmake_minimum_required(VERSION 3.8)
project(agentswarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentDistance.msg"
  "msg/Heartbeat.msg"
  "msg/ElectionStatus.msg"
  "msg/JobAssignment.msg"
  "msg/JobBid.msg"
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(agent_node src/agent_node.cpp)
ament_target_dependencies(agent_node rclcpp std_msgs)



add_executable(job_publisher src/job_publisher.cpp)
ament_target_dependencies(job_publisher rclcpp std_msgs)
rosidl_get_typesupport_target(job_publisher_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(job_publisher "${job_publisher_typesupport_target}")
install(TARGETS job_publisher DESTINATION lib/${PROJECT_NAME})

# Use the newer recommended way of linking with the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(agent_node "${cpp_typesupport_target}")

install(TARGETS
  agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
