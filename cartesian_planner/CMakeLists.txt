cmake_minimum_required(VERSION 3.5)
project(cartesian_planner)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cartesian_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/error_codes.cpp
  src/utility.cpp
  src/path.cpp
  src/time_scaling.cpp
  src/trajectory.cpp
  src/smoothing.cpp
  src/cartesian_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
  kdl_parser
  tf2_eigen_kdl
  tf2_eigen
  trajectory_msgs
  cartesian_planning_msgs
  Eigen3
)

# Export include dirs and libraries
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  urdf
  kdl_parser
  tf2_eigen_kdl
  tf2_eigen
  trajectory_msgs
  cartesian_planning_msgs
  Eigen3
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
