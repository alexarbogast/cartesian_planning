cmake_minimum_required(VERSION 3.0.2)
project(cartesian_planning_examples)

if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  control_msgs
  trajectory_msgs
  actionlib
  cartesian_planning_server
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  control_msgs
  trajectory_msgs
  actionlib
  cartesian_planning_server
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(cartesian_planning_demo src/cartesian_planning_demo.cpp)
target_link_libraries(cartesian_planning_demo ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/cartesian_planning_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
