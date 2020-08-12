#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class cSpheroVelControl(object):
    def __init__(self):
        self._vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.linearspeed_ = 0.2
        self.angularspeed_ = 0.5
        
    def move_robot(self, direction):
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
        
        self._vel_pub_.publish(msg)

    def shutdown(self):
        
        rospy.loginfo('shutdown time!')
        
        self.is_running_ = False
        self.move_robot('stop')

    def spin(self):
        rate = rospy.Rate(1)
        while self.is_running_:
            self.move_robot(direction='forwards')
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher_node')
    controller = cSpheroVelControl()
    
    rospy.on_shutdown(controller.shutdown)

    controller.spin()
    
