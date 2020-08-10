#! /usr/bin/env python
import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes 

def main():
    rospy.init_node('exercise_4_3_client')
    rospy.wait_for_service('/move_bb8_in_circle')
    service = rospy.ServiceProxy('/move_bb8_in_circle', Empty)
    service()

if __name__ == '__main__':
    main()

