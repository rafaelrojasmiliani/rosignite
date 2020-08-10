#! /usr/bin/env python
import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes 
                                                                                         # generated from MyCustomServiceMessage.srv.


class cServiceServer(object):
    def __init__(self):
        self.srv_ = rospy.Service('/my_service', MyCustomServiceMessage , self.service) 

    def service(self, *data):
        my_response = MyCustomServiceMessageResponse()
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
