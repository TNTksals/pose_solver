cmake_minimum_required(VERSION 3.10)
project(rm_pose_solver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  dynamic_reconfigure
  geometry_msgs
  visualization_msgs
  # image_transport
  cv_bridge
  image_geometry
  nodelet
  tf
  tf2
  tf2_geometry_msgs
  rm_msgs
  rm_vision
  message_generation
)

message("OpenCV version:" ${OpenCV_VERSION})

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  dynamic_reconfigure
  # message_generation
  rm_msgs
  rm_vision
  std_msgs
  pluginlib
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pose_solver_plugin
  src/pose_solver.cpp
)

target_link_libraries(pose_solver_plugin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

