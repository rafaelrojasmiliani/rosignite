#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mytools import cTopicReader

class cImuReader(cTopicReader):
    def __init__(self, _topic_name = '/sphero/imu/data3'):
        cTopicReader.__init__(self, _topic_name, Imu)

#    def in_spin(self):
#        rospy.loginfo('[imu]'+str(self.get_data()))

if __name__ == '__main__':
    rospy.init_node('imu_reader')
    reader = cImuReader()
    reader.spin()
