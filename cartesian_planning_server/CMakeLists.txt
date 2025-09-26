cmake_minimum_required(VERSION 3.8)
project(cartesian_planning_server)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cartesian_planner REQUIRED)
find_package(cartesian_planning_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/cartesian_planning_server.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_eigen
  cartesian_planner
  cartesian_planning_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
