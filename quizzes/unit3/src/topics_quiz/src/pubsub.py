
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class cPubSub(object):
    def __init__(self):
        self.sub_ = rospy.Subscriber("/kobuki/laser/scan", LaserScan, self.control_action)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def control_action(self, *data):
        msg = data[0]
        print('len of the array is {:d}'.format(len(msg.ranges)))
        msg = Twist()
        v = np.random.rand(3) 
        w = np.random.rand(3) 
        for coord, vi, wi in zip('xyz', v, w):
            setattr(msg.linear, coord, vi)
            setattr(msg.angular, coord, wi)
        pub.publish(msg)


def main():
    rospy.init_node('listener', anonymous=True)

    controller = cPubSub()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
