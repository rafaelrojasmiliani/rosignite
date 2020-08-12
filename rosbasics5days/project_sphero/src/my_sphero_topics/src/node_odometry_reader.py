#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from mytools import cTopicReader

class cOdometryReader(cTopicReader):
    def __init__(self, _topic_name = '/odom', **kwargs):
        cTopicReader.__init__(self, _topic_name, Odometry, **kwargs)

#    def in_spin(self):
#        rospy.loginfo('[odometry]'+str(self.get_data()))

if __name__ == '__main__':
    rospy.init_node('odometry_reader')
    reader = cOdometryReader()
    reader.spin()
