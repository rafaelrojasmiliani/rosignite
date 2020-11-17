cmake_minimum_required(VERSION 3.0.2)
project(my_gurdy_description)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
)
catkin_package(
  CATKIN_DEPENDS rospy 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
