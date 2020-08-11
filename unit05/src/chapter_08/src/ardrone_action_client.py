#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback


class cActionClient(object):
    def __init__(self):
        self.images_number_ = 1
        self.action_ = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)

    def feedback_callback(self, _feedback):
        rospy.loginfo('[Feedback] image n{:d} received'.format(self.images_number_))
        self.images_number_ += 1

    def execute_action_and_wait(self, _nsecods):
        self.execute_action(_nsecods)
        self.action_.wait_for_server()

    def execute_action_wait_and_stop(self):
        ''' Example to show that an action is asynchronous'''
        goal = ArdroneGoal()
        goal.nseconds = 30
        action = self.action_
        action.send_goal(goal, feedback_cb=self.feedback_callback)
        time.sleep(0.5)
        rospy.loginfo('It is asynchronous!!')
        status = action.get_state()
        rospy.loginfo('The status is = {:d}'.format(status))
        action.cancel_goal()  # would cancel the goal 3 seconds after starting
        status = action.get_state()
        rospy.loginfo('just after cancel = {:d}'.format(status))
        time.sleep(2)
        status = action.get_state()
        rospy.loginfo('The status is = {:d}'.format(status))
        

def main():
    rospy.init_node('drone_action_client')
    action_client = cActionClient()
    action_client.execute_action_wait_and_stop()

if __name__ == '__main__':
    main()
