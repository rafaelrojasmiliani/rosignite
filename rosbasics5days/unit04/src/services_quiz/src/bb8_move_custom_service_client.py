#! /usr/bin/env python
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest

def main():
    rospy.init_node('services_quiz_client')
    rospy.wait_for_service('/move_bb8_in_square_custom')
    service = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage)
    req = BB8CustomServiceMessageRequest()
    req.side = 5
    req.repetitions = 2
    service(req)
    req.side = 2
    req.repetitions = 1
    service(req)

if __name__ == '__main__':
    main()


