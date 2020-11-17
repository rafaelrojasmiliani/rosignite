cmake_minimum_required(VERSION 2.8.3)
project(my_robot_control)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
