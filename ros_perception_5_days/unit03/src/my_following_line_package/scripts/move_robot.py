#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class MoveRobot:

    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)

    def spin(self):
        twist_object = Twist()
        twist_object.angular.z = 0.5
        rate = rospy.Rate(5)
        while not self.ctrl_c:
            self.cmd_vel_pub.publish(twist_object)
            rate.sleep()

    def shutdownhook(self):
        self.clean_up()
        rospy.loginfo("shutdown time!")
        self.ctrl_c = True


def main():
    rospy.init_node('move_robot_node', anonymous=True)

    robot = MoveRobot()
    robot.spin()


if __name__ == '__main__':
    main()
