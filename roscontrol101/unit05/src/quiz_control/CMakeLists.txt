cmake_minimum_required(VERSION 2.8.3)
project(quiz_control)
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
