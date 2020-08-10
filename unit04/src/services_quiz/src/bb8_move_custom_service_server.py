#! /usr/bin/env python
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse


class cServiceServer(object):
    def __init__(self):
        self.srv_ = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , self.service) 

    def service(self, *data):
        request = data[0]
        my_response = BB8CustomServiceMessageResponse()
        if request.duration > 5.0:
            my_response.success = True
        else:
            my_response.success = False
        return  my_response 



def main():
    rospy.init_node('service_client') 
    server = cServiceServer()
    rospy.spin() # mainta

if __name__ == '__main__':
    main()

