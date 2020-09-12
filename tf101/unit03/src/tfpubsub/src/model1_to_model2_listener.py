#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg


class cTfSubscriber:
    def __init__(self, _stalker, _victim):
        rospy.init_node('tf_listener_turtle')
        self.tf_lister_ = tf.TransformListener()
        self.stalker_ = _stalker
        self.stalker_frame_ = '/'+_stalker
        self.victim_ = _victim
        self.victim_frame_ = '/'+_victim
        self.ctrl_c_ = False

        self.staker_vel_pub_ = rospy.Publisher(
            _stalker+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

        self.rate_ = rospy.Rate(10.0)
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        print "shutdown time! Stop the robot"
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        turtle_vel.publish(cmd)
        self.ctrl_c_ = True

    def spin(self):
        while not self.ctrl_c_:
            try:
                (trans, rot) = self.tf_lister_.lookupTransform(
                    self.stalker_frame_, self.victim_frame_, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.staker_vel_pub_.publish(cmd)

            rate.sleep()

def main(*argv):
    argv=argv[1:3]
    subs = cTfSubscriber(*argv)
    subs.spin()



if __name__ == '__main__':

    if len(sys.argv) < 3:
        print("usage: turtle_tf_listener.py follower_model_name model_to_be_followed_name")
        sys.exit()

    main(*sys.argv[1:])

