#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class cPubSub(object):
    def __init__(self):
        self.sub_ = rospy.Subscriber("/kobuki/laser/scan", LaserScan, self.control_action)
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def control_action(self, *data):
        feed_back_msg = data[0]

        front_lecture = np.min(feed_back_msg.ranges[240:480])
        left_lecture = np.min(feed_back_msg.ranges[480:])
        right_lecture = np.min(feed_back_msg.ranges[:240])

        msg = Twist()
        for coord in 'xyz':
            setattr(msg.linear, coord, 0.0)
            setattr(msg.angular, coord, 0.0)

        if front_lecture > 1.0:
            msg.linear.x = 1.0
        elif front_lecture <= 1.0:
            msg.angular.z = 1.0
        elif right_lecture < 1.0:
            msg.angular.z = 1.0
        elif left_lecture < 1.0:
            msg.angular.z = 1.0

        self.pub_.publish(msg)


def main():
    rospy.init_node('listener', anonymous=True)

    controller = cPubSub()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
