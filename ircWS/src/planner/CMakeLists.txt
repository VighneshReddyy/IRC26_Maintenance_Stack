cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

include_directories(
  include
  /opt/ros/humble/include
)

add_library(shared_state src/shared_state.cpp)
ament_target_dependencies(shared_state
  rclcpp
  std_msgs
  custom_msgs
)

add_executable(stack_node src/stack.cpp)
ament_target_dependencies(stack_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  custom_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(stack_node shared_state)

add_executable(relay src/relay.cpp)
ament_target_dependencies(relay
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)
target_link_libraries(relay shared_state)

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

add_executable(motorcontroller src/motorcontroller.cpp)
ament_target_dependencies(motorcontroller
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(motorcontroller shared_state)

install(TARGETS
  shared_state
  stack_node
  relay
  imu_conversion_node
  motorcontroller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/mode_switch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
