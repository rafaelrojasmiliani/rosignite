#! /usr/bin/env python
import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes 
from geometry_msgs.msg import Twist
                                                                                         # generated from MyCustomServiceMessage.srv.


class cServiceServer(object):
    def __init__(self):
        self.srv_ = rospy.Service('/move_bb8_in_circle_custom', MyCustomServiceMessage , self.service) 

        self.rate_ = rospy.Rate(1)

        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.cmd_vel_stop_ = Twist()
        self.cmd_vel_move_ = Twist()
        self.cmd_vel_move_.linear.x = 0.2
        self.cmd_vel_move_.angular.z = 0.2


    def service(self, *data):
        request = data[0]

        self.pub_.publish(self.cmd_vel_move_)
        for _ in range(request.duration):
            self.rate_.sleep()


        self.pub_.publish(self.cmd_vel_stop_)

        my_response = MyCustomServiceMessageResponse()
        my_response.success = True
        return  my_response 



def main():
    rospy.init_node('exercise_4_3_server') 
    server = cServiceServer()
    rospy.spin() # mainta

if __name__ == '__main__':
    main()

