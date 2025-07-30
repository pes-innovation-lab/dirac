cmake_minimum_required(VERSION 3.8)
project(dirac_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dirac_lib REQUIRED)
find_package(dirac_msgs REQUIRED)

include_directories(
  include
)

add_executable(agent_node src/agent_node.cpp)
target_link_libraries(agent_node)
target_include_directories(agent_node PRIVATE include)
ament_target_dependencies(agent_node rclcpp dirac_lib dirac_msgs)

add_executable(job_publisher_node ../dirac_lib/src/job_publisher_node.cpp)
target_link_libraries(job_publisher_node)
target_include_directories(job_publisher_node PRIVATE ../dirac_lib/include)
ament_target_dependencies(job_publisher_node rclcpp dirac_msgs)

install(TARGETS
  agent_node
  job_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
