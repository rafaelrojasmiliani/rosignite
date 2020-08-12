cmake_minimum_required(VERSION 2.8.3)
project(actions_quiz)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
