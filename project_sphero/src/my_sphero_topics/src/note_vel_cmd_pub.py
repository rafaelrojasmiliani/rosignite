#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mytools import cTopicWriter


class cSpheroVelControl(cTopicWriter):
    def __init__(self, _topic_name):
        cTopicWriter.__init__(self, _topic_name, Twist)
        
    def build_message(self):
        msg = Twist()
        if direction == 'forwards':
            msg.linear.x = self.linearspeed_
        elif direction == 'right':
            msg.angular.z = self.angularspeed_
        elif direction == 'left':
            msg.angular.z = -self.angularspeed_
        elif direction == 'backwards':
            msg.linear.x = -self.linearspeed_
        else:
            pass
        return msg

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher_node')
    controller = cSpheroVelControl()
    controller.spin()
    
