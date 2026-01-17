cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# =====================
# Shared library
# =====================
add_library(shared_state src/shared_state.cpp)
ament_target_dependencies(shared_state
  rclcpp
  std_msgs
  custom_msgs
)

# =====================
# Main planner
# =====================
add_executable(planner_node
  src/main.cpp
  src/irc_planner.cpp
)
ament_target_dependencies(planner_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  custom_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(planner_node
  shared_state
  ${PCL_LIBRARIES}
)

# =====================
# Relay node
# =====================
add_executable(relay_node src/relay_node.cpp)
ament_target_dependencies(relay_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)
target_link_libraries(relay_node shared_state)

# =====================
# IMU conversion
# =====================
add_executable(imu_conversion_node src/imu_conversion.cpp)
ament_target_dependencies(imu_conversion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  custom_msgs
)
target_link_libraries(imu_conversion_node shared_state)

# =====================
# Motor controller
# =====================
add_executable(motorcontroller src/motorcontroller.cpp)
ament_target_dependencies(motorcontroller
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(motorcontroller shared_state)

# =====================
# DITCH + SAFETY FUSED NODE (PointCloud-based)
# =====================
add_executable(ditch_detector_node src/ditch_detector.cpp)
ament_target_dependencies(ditch_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
)
target_link_libraries(ditch_detector_node
  ${PCL_LIBRARIES}
)

# =====================
# Install
# =====================
install(TARGETS
  shared_state
  planner_node
  relay_node
  imu_conversion_node
  motorcontroller
  ditch_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/mode_switch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
