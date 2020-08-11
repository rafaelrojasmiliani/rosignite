#! /usr/bin/env python
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction


class cFibonacciCalculator(object):

  # create messages that are used to publish feedback/result
    def __init__(self):
        self._feedback_ = FibonacciFeedback()
        action = actionlib.SimpleActionServer(
            "fibonacci_as", FibonacciAction, self.goal_callback, False)
        action.start()
        self._as_ = action

    def compute_fibonacci(self, _order):
        i0 = self._feedback_.sequence[0]
        i1 = self._feedback_.sequence[1]
        i_ant = i0
        feedback_strening_rate = rospy.Rate(1)
        action = self._as_
        feedback = self._feedback_
        for i in xrange(i1, _order):
            if self._as_.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                action.set_preempted()
                return False
            current_number = i + i_ant
            feedback.sequence.append(current_number)
            action.publish_feedback(feedback)
            feedback_strening_rate.sleep()
        return True


    def goal_callback(self, goal):
        self._feedback_.sequence = []
        self._feedback_.sequence.append(0)
        self._feedback_.sequence.append(1)
        message = 'Executing, creating fibonacci sequence of order {} with seeds {}, {}'.format(
            goal.order, self._feedback_.sequence[0], self._feedback_.sequence[1])
        rospy.loginfo(message)

        success = self.compute_fibonacci(goal.order)

        if success:
            result = FibonacciResult()
            result.sequence = self._feedback_.sequence
            rospy.loginfo(
                'Succeeded calculating the Fibonacci of order %i' % fibonacciOrder)
            self._as_.set_succeeded(self._result)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('fibonacci')
    fibo = cFibonacciCalculator()
    fibo.spin()
