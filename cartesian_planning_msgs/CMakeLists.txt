cmake_minimum_required(VERSION 3.0.2)
project(cartesian_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

add_message_files(DIRECTORY msg FILES ErrorCodes.msg)
add_service_files(DIRECTORY srv FILES PlanCartesianTrajectory.srv)
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS
  message_runtime
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
