#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mytools import cTopicReader

class cImuReader(cTopicReader):
    def __init__(self, _topic_name = '/odom'):
        cTopicReader.__init__(_topic_name, Odometry)

    def in_spin(self):
        rospy.logdebug('[odometry]'+str(self.get_data()))

if __name__ == '__main__':
    rospy.init_node('imu_reader')
    reader = cImuReader()
    reader.spin()