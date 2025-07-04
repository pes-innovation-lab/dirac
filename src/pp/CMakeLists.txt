cmake_minimum_required(VERSION 3.8)
project(pp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(talk REQUIRED)


# define the executable
add_executable(agent_node src/agent_node.cpp)
ament_target_dependencies(agent_node
  rclcpp
  std_msgs
  geometry_msgs
  talk
)

install(TARGETS
  agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
