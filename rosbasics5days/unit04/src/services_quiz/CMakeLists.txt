cmake_minimum_required(VERSION 2.8.3)
project(services_quiz)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
add_service_files(
  FILES
  BB8CustomServiceMessage.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
