#! /usr/bin/env python
import rospy
import actionlib
from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import numpy as np


class cDroneMission(object):

  # create messages that are used to publish feedback/result
    def __init__(self):
        action = actionlib.SimpleActionServer(
            '/action_custom_msg_as', CustomActionMsgAction, self.goal_callback, False)
        action.start()
        self._as_ = action


    def goal_callback(self, _goal):

        feedback = CustomActionMsgFeedback()
        feedback.feedback = _goal.goal

        if _goal.goal == 'TAKEOFF ':
            activity = self.takeoff_drone
        elif _goal.goal == 'LAND':
            activity = self.land_drone
        else:
            assert 0

        action = self._as_
        rate =  rospy.Rate(1)
        activity()
        while not rospy.is_shutdown():
            action.publish_feedback(feedback)
            rate.sleep()

        result = CustomActionMsgResult()
        action.set_succeeded(result)

    def spin(self):
        rospy.spin()

    def takeoff_drone(self):
        pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        msg = Empty()
        rate = rospy.Rate(100)
        for _ in range(50):
            pub.publish(msg)
            rate.sleep()

        return True

    def land_drone(self):
        pub = rospy.Publisher('/drone/land', Empty, queue_size=10)
        msg = Empty()
        rate = rospy.Rate(100)
        for _ in range(50):
            pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('actions_quiz')
    drone = cDroneMission()
    drone.spin()

