#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np


class cPicking(smach.State):
    """Example of smach state"""

    def __init__(self):
        """  How to construc an state. The inputs of a state constructor are:
            Definition (state outcomes): We call state outcomes all the
            possible returns from smach.State.execute. State outcomes are strings.
            - outcomes: (list of strings) custom outcomes for this state.
            - input_keys: (list of strings) The userdata keys from which this
              state might read at runtime.
            -  output_keys: (list of strings) The userdata keys to which this
               state might write at runtime.
            -  io_keys: (list of strings) The userdata keys to which this state
               might write or from which it might read at runtime.
        """
        smach.State.__init__(self, outcomes=['success', 'fail'])

        self.counter_ = 0

        self.success_probability_ = 0.6

    def execute(self, _ud):
        """ Main function executed in the state"""

        self.counter_ += 1
        print('cPicking')

        if np.random.rand() < self.success_probability_:
            return 'success'

        return 'fail'


class cPlacing(smach.State):
    """Example of smach state"""

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

        self.counter_ = 0

    def execute(self, _ud):
        print('cPlacing')
        """ Code that its executed when the robot is in this state"""
        return 'success'


def main():
    rospy.init_node('my_sm')

    sm = smach.StateMachine(outcomes=['success', 'fail'])

    with sm:
        # definition (transition): A dictionary mapping state outcomes to other
        # state labels (class.__name__) or container outcomes (outcomes in the
        # smach.StateMachine constructor.
        smach.StateMachine.add(
            'cPicking',
            cPicking(),
            transitions={
                'success': 'cPlacing',
                'fail': 'cPicking'
            })
        smach.StateMachine.add(
            'cPlacing',
            cPicking(),
            transitions={
                'success': 'success',
                'fail': 'cPlacing'
            })

    outcome = sm.execute()


if __name__ == "__main__":
    main()
