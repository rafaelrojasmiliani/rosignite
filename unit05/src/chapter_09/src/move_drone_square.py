#! /usr/bin/env python
import rospy
import actionlib
from actionlib.msg import TestFeedback, TestResult, TestAction
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import numpy as np


class cDroneMission(object):

  # create messages that are used to publish feedback/result
    def __init__(self):
        self._feedback_ = TestFeedback()
        action = actionlib.SimpleActionServer(
            'move_drone_square_as', TestAction, self.goal_callback, False)
        action.start()
        self.cmd_vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._as_ = action
        self.time_step_ = 0.01
        self.rate_ = rospy.Rate(1.0/self.time_step_)
        self.execution_time_ = 0.0

    def mission(self, _size):
        t0 = time.time()

        rospy.loginfo('Take off drone')
        self.takeoff_drone()
        action = self._as_
        feedback = self._feedback_

        for _ in range(4):
            action.publish_feedback(feedback)
            self.move_forward(_size)
            self.turn_90()


        self.land_drone()
        t1 = time.time()
        self.execution_time_ = t1-t0
        return True


    def goal_callback(self, _goal):

        success = self.mission(_goal.goal)

        if success:
            result = TestResult()
            result.result = self.execution_time_
            self._as_.set_succeeded(result)

    def spin(self):
        rospy.spin()

    def takeoff_drone(self):
        pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        msg = Empty()
        for _ in range(40):
            pub.publish(msg)
            self.rate_.sleep()

        return True

    def move_forward(self, _distance):
        rospy.loginfo('move forward')
        msg = Twist()
        vel = 0.2
        msg.linear.x = vel
        distance_traveled = 0
        while distance_traveled < _distance:
            self.cmd_vel_pub_.publish(msg)
            self.rate_.sleep()
            distance_traveled += vel *self.time_step_
        msg.linear.x = 0
        for _ in range(20):
            self.cmd_vel_pub_.publish(msg)
            self.rate_.sleep()

    def turn_90(self):
        rospy.loginfo('turning 90 degress')
        msg = Twist()
        vel = 0.2
        msg.angular.z = vel
        distance_traveled = 0
        while distance_traveled < np.pi/2:
            self.cmd_vel_pub_.publish(msg)
            self.rate_.sleep()
            distance_traveled += vel *self.time_step_
        msg.angular.z = 0.0
        for _ in range(40):
            self.cmd_vel_pub_.publish(msg)
            self.rate_.sleep()

    def land_drone(self):
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = 0
        msg.linear.y = 0
        elapsed_time = 0.0
        self.cmd_vel_pub_.publish(msg)
        self.rate_.sleep()
        pub = rospy.Publisher('/drone/land', Empty, queue_size=10)
        msg = Empty()
        pub.publish(msg)
        for _ in range(10):
            pub.publish(msg)
            self.rate_.sleep()


if __name__ == '__main__':
    rospy.init_node('drone_move_square')
    drone = cDroneMission()
    drone.spin()

