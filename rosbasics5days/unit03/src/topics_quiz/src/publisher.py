#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

def main():

    rospy.init_node('topic_publisher') 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        v = np.random.rand(3) 
        w = np.random.rand(3) 
        for coord, vi, wi in zip('xyz', v, w):
            setattr(Twist.linear, coord, vi)
            setattr(Twist.linear, coord, wi)

        rate.sleep()


    
