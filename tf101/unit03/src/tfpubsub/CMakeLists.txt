cmake_minimum_required(VERSION 3.0.2)
project(tfpubsub)
find_package(catkin REQUIRED COMPONENTS
  rospy
  turtle_tf_3d
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
