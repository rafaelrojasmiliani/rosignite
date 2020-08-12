#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse 

def main():
    rospy.init_node('exervice_4_2_client')
    rospy.wait_for_service('/move_bb8_in_circle')
    service = rospy.ServiceProxy('/move_bb8_in_circle', Empty)
    service()

if __name__ == '__main__':
    main()
