cmake_minimum_required(VERSION 3.5)
project(cartesian_planning_examples)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cartesian_planning_msgs REQUIRED)
find_package(cartesian_planning_server REQUIRED)

add_executable(cartesian_planning_demo src/cartesian_planning_demo.cpp)
ament_target_dependencies(cartesian_planning_demo
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  cartesian_planning_msgs
  cartesian_planning_server
)

install(TARGETS cartesian_planning_demo
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/cartesian_planning_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
