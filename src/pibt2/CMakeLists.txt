cmake_minimum_required(VERSION 3.16)
project(pibt2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

add_executable(pibt_agent_node pibt_agent_node.cpp)
ament_target_dependencies(pibt_agent_node rclcpp std_msgs)
target_link_libraries(pibt_agent_node nlohmann_json::nlohmann_json)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_package() 
