#! /usr/bin/env python
import rospy
import time
import tf
from turtle_tf_3d.get_model_gazebo_pose import GazeboModel


class cTfPulbisher:
    def __init__(self, _name_list):
        rospy.init_node('publisher_of_tf_node', anonymous=True)
        self.tfbr_ = tf.TransformBroadcaster()
        self.rate_ = rospy.Rate(5)
        self.name_list_ = _name_list
        self.gzb_models_ = GazeboModel(_name_list)

    def spin(self):

        for robot_name in self.name_list_:
            pose_now = self.gzb_models_.get_model_pose(robot_name)

        while not rospy.is_shutdown():
            for robot_name in self.name_list_:
                pose_now = self.gzb_models_.get_model_pose(robot_name)
                if pose_now:
                    pos = [getattr(pose_now.position, component)
                           for component in 'xyz']
                    ori = [getattr(pose_now.orientation, component)
                           for component in 'xyzw']
                    self.tfbr_.sendTransform(pos,
                                     ori,
                                     rospy.Time.now(),
                                     robot_name,
                                     "/world")
        self.rate_.sleep()


def main():
    robot_name_list = ["turtle1", "turtle2"]

    tfpub = cTfPulbisher(robot_name_list)

    tfpub.spin()


if __name__ == '__main__':
    main()
