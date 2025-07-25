cmake_minimum_required(VERSION 3.8)
project(dirac_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dirac_lib REQUIRED)

# Include headers
include_directories(
  include
)

# Executable
add_executable(agent_node
  src/agent_node.cpp
)

ament_target_dependencies(agent_node
  rclcpp
  dirac_lib
)

# Install executable
install(TARGETS agent_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (optional)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
