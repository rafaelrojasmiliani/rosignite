#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import numpy as np

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
        self.time_step_ = 0.1
        self.rate_ = rospy.Rate(1.0/self.time_step_)
        self.cmd_vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
        action.wait_for_server()
        action.send_goal(goal, feedback_cb=self.feedback_callback)
        for _ in range(10):
            self.rate_.sleep()

        self.takeoff_drone()

        action_state = action.get_state()
        while action_state < SimpleGoalState.DONE:
            self.move_drone()
            action_state = action.get_state()

        self.land_drone()

        action.cancel_goal()
    
    def takeoff_drone(self):
        pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        msg = Empty()
        pub.publish(msg)
        for _ in range(10):
            pub.publish(msg)
            self.rate_.sleep()

    def move_drone(self):
        msg = Twist()
        msg.angular.z = 1
        msg.linear.x = np.rand.random()*0.1
        msg.linear.y = np.rand.random()*0.1
        elapsed_time = 0.0
        self.cmd_vel_pub_.publish(msg)
        self.rate_.sleep()

    def land_drone(self):
        pub = rospy.Publisher('/drone/land', Empty, queue_size=10)
        msg = Empty()
        pub.publish(msg)
        for _ in range(10):
            pub.publish(msg)
            self.rate_.sleep()

        

def main():
    rospy.init_node('drone_action_client')
    action_client = cActionClient()
    action_client.execute_action_wait_and_stop()

if __name__ == '__main__':
    main()
