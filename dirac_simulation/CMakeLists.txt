cmake_minimum_required(VERSION 3.8)
project(dirac_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dirac_msgs REQUIRED)

# Add executable
add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller 
  rclcpp 
  geometry_msgs 
  dirac_msgs
)

# Install targets
install(TARGETS
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
