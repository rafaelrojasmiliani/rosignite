#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse 
from geometry_msgs.msg import Twist

class cSrvServer(object):

    def __init__(self):

        self.start_srv_ = rospy.Service('/move_bb8_in_circle', Empty, self.start_service)
        self.stop_srv_ = rospy.Service('/stop_bb8', Empty, self.stop_service)

        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.cmd_buff_ = Twist()

    def start_service(self, *args):
        command = self.cmd_buff_
        command.linear.x = 1
        command.angular.z = 1
        self.pub_.publish(command)
        result = EmptyResponse()
        return result

    def stop_service(self, *args):
        command = self.cmd_buff_
        command.x = 0
        command.z = 0
        self.pub_.publish(command)
        result = EmptyResponse()
        return result

def main():
    rospy.init_node('exercise_4_2_server') 

    srvsrv = cSrvServer()

    rospy.spin()
    

if __name__ == '__main__':
    main()




