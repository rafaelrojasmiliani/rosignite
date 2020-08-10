import rospy
from std_srvs.srv import Empty, EmptyResponse 

class cSrvServer(object):

    def __init__(self):

        self.srv_ = rospy.Service('/my_service', Empty, self.service)

    def service(self, *args):
        result = EmptyResponse()
        return result


def main():
    rospy.init_node('service_server') 

    srvsrv = cSrvServer()

    rospy.spin()
    

if __name__ == '__main__':
    main()



