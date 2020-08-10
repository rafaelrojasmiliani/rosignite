#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from iri_wam_reproduce_trajectory.srv import ExecTraj, ExecTrajRequest
import sys
import rospkg


class cNode(object):
    def __init__(self):
        self.service_ = rospy.ServiceProxy('/execute_trajectory', ExecTraj)
        rospack = rospkg.RosPack()
        self.trj_file_ = rospack.get_path('iri_wam_reproduce_trajectory') + "/config/get_food.txt"

    def main(self):
        req = ExecTrajRequest()
        req.file = self.trj_file_
        result = traj_by_name_service(req)

        

def main():
    rospy.init_node('service_client')
    rospy.wait_for_service('/execute_trajectory')

    node = cNode()

    node.main()

    

if __name__ == '__main__':
    main()
