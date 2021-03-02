#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob


class SimpleTracker:
    def __init__(self):
        self.xpos_ = (180+220)*0.5
        self.control_pub_ = rospy.Publisher(
            '/mira/commands/velocity', Twist, queue_size=1)
        self.blob_subs_ = rospy.Subscriber('/blobs', Blobs, self.get_blobs)

    def get_blobs(self, _msg):
        for blob in _msg.blobs:
            if blob.name == "RedBall":
                self.xpos_ = blob.x
                break

    def spin(self):
        twist = Twist()
        desired = (180+220)*0.5
        gain = 0.001

        while not rospy.is_shutdown():
            # turn if we hit the line
            twist.linear.x = 0.0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = gain*(desired - self.xpos_)
            self.control_pub_.publish(twist)
            rospy.sleep(0.005)


def main():
    rospy.init_node("track_blob_color_node", log_level=rospy.WARN)
    tracker = SimpleTracker()
    tracker.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
