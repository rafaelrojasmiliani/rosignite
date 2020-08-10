#! /usr/bin/env python
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist
                                                                                         
class cServiceServer(object):
    def __init__(self):
        self.srv_ = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , self.service) 
        self.rate_ = rospy.Rate(1)
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def service(self, *_data):
        request = _data[0]
        time_size_scale = 5
        repetitions = request.repetitions
        side = request.side*time_size_scale

        for _ in range(repetitions):
            for _ in range(4):
                self.move_linear(0.5)
                self.rate_(side)
                self.move_angular(0.5)
                self.rate_(side)

        my_response = BB8CustomServiceMessageResponse()
        my_response.success = True
        return  my_response 

    def move_linear(self, _vx):
        command = Twist()
        command.linear.x = vx
        self.pub_.publish(command)

    def move_angular(self, _wz):
        command = Twist()
        command.angular.z = _wz
        self.pub_.publish(command)


def main():
    rospy.init_node('service_client') 
    server = cServiceServer()
    rospy.spin() # mainta

if __name__ == '__main__':
    main()

