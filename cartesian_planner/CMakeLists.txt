cmake_minimum_required(VERSION 3.0.2)
project(cartesian_planner)

if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  kdl_parser
  eigen_conversions
  trajectory_msgs
  cartesian_planning_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    urdf
    kdl_parser
    eigen_conversions
    trajectory_msgs
    cartesian_planning_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/error_codes.cpp
  src/utility.cpp
  src/path.cpp
  src/time_scaling.cpp
  src/trajectory.cpp
  src/cartesian_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

# add_executable(my_test src/test.cpp)
# target_link_libraries(my_test ${catkin_LIBRARIES} ${PROJECT_NAME})
