cmake_minimum_required(VERSION 3.16)
project(pibt2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)


add_executable(pibt_agent_node pibt_agent_node.cpp)  # use `src/` if that’s where the file is
ament_target_dependencies(pibt_agent_node rclcpp std_msgs)
target_link_libraries(pibt_agent_node nlohmann_json::nlohmann_json)

install(TARGETS
  pibt_agent_node
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()


