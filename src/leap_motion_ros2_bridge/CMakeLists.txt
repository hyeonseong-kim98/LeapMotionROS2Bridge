cmake_minimum_required(VERSION 3.8)
project(leap_motion_ros2_bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(leap_motion_interface REQUIRED)
# Leap Motion SDK
set(LEAP_MOTION_INCLUDE_DIR "/usr/lib/ultraleap-hand-tracking-service/leapc_cffi")
set(LEAP_MOTION_LIBRARY_DIR "/usr/lib/ultraleap-hand-tracking-service/leapc_cffi")

# Include directories
include_directories(
  include/leap_motion_ros2_bridge
  ${LEAP_MOTION_INCLUDE_DIR}
)

# Link directories
link_directories(${LEAP_MOTION_LIBRARY_DIR})

# Add executable
add_executable(leap_motion_ros2_bridge src/leap_motion_ros2_bridge.cpp)

# Link libraries
ament_target_dependencies(leap_motion_ros2_bridge rclcpp std_msgs leap_motion_interface)
target_link_libraries(leap_motion_ros2_bridge LeapC)

# Install targets
install(TARGETS
  leap_motion_ros2_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()