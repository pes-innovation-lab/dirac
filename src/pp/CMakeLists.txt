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

# modern interface linking
rosidl_get_typesupport_target(cpp_typesupport_target "talk" "rosidl_typesupport_cpp")
target_link_libraries(agent_node "${cpp_typesupport_target}")

install(TARGETS
  agent_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
