#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
import smach

class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

class cActionClient(object):
    def __init__(self):
        self.images_number_ = 1
        self.action_ = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
        self.rate_ = rospy.Rate(10)

    def feedback_callback(self, _feedback):
        rospy.loginfo('[Feedback] image n{:d} received'.format(self.images_number_))
        self.images_number_ += 1

    def execute_action_and_wait(self, _nsecods):
        self.execute_action(_nsecods)
        self.action_.wait_for_server()

    def execute_and_wait_for_result(self):
        ''' Example to show that an action is asynchronous'''
        goal = ArdroneGoal()
        goal.nseconds = 30
        action = self.action_
        action.wait_for_server()
        action.send_goal(goal, feedback_cb=self.feedback_callback)

        action_state = action.get_state()

        while action_state < SimpleGoalState.DONE:
            rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
            self.rate_.sleep()
            action_state = action.get_state()
            rospy.loginfo("action state: {:d}".format(action_state))
        

def main():
    rospy.init_node('drone_action_client')
    action_client = cActionClient()
    action_client.execute_and_wait_for_result()

if __name__ == '__main__':
    main()
