cmake_minimum_required(VERSION 2.8.3)
project(my_sphero_topics)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  #  actionlib_msgs
)

#add_action_files(
#    FILES
#    CustomActionMsg.action
#)

generate_messages(
    DEPENDENCIES
    std_msgs #actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
