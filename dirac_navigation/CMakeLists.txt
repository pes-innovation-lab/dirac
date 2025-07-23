cmake_minimum_required(VERSION 3.8)
project(dirac_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dirac_msgs REQUIRED)

# Create library
add_library(navigation_controller SHARED
  src/navigation_controller.cpp
  src/strategies/movement_strategy.cpp
  src/strategies/forward_movement.cpp
  src/strategies/backward_movement.cpp
  src/strategies/left_movement.cpp
  src/strategies/right_movement.cpp
  src/strategies/movement_strategy_factory.cpp
)

ament_target_dependencies(navigation_controller 
  rclcpp 
  geometry_msgs 
  dirac_msgs
)

# Include directories
target_include_directories(navigation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add main executable that uses the library
add_executable(discrete_navigation_controller src/discrete_navigation_controller_node.cpp)
ament_target_dependencies(discrete_navigation_controller 
  rclcpp 
  geometry_msgs 
  dirac_msgs
)
target_link_libraries(discrete_navigation_controller navigation_controller)

# Install targets
install(TARGETS
  navigation_controller
  EXPORT navigation_controllerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  discrete_navigation_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export targets
ament_export_targets(navigation_controllerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs dirac_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
