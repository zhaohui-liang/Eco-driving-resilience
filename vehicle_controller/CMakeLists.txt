cmake_minimum_required(VERSION 3.8)
project(vehicle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Source files
set(SOURCES
  src/vehicle_node.cpp
  src/vehicle_controller.cpp
  src/signal_io.cpp
)

# Executable
add_executable(vehicle_controller_node ${SOURCES})
ament_target_dependencies(vehicle_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  novatel_oem7_msgs
)

target_link_libraries(vehicle_controller_node
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS vehicle_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
