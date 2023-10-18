cmake_minimum_required(VERSION 3.0.2)
project(cartesian_planning_server)

if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  cartesian_planner
)

add_service_files(DIRECTORY srv FILES PlanCartesianTrajectory.srv)
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  message_runtime
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  cartesian_planner
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/cartesian_planning_server.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install executables
install(TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
