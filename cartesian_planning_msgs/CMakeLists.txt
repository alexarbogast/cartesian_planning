cmake_minimum_required(VERSION 3.5)
project(cartesian_planning_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/ErrorCodes.msg"
)

set(srv_files
  "srv/PlanCartesianTrajectory.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
