cmake_minimum_required(VERSION 2.8.3)
project(sia10f_description)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
