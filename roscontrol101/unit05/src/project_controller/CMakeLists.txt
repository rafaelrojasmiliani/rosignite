cmake_minimum_required(VERSION 2.8.3)
project(project_controller)

find_package(catkin REQUIRED COMPONENTS controller_interface hardware_interface
                                        pluginlib roscpp)

add_library(project_controller_lib src/my_controller.cpp)
target_link_libraries(project_controller_lib ${catkin_LIBRARIES})
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
