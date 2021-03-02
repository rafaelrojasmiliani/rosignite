#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveRobot


class LineFollower(object):

    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge_object = CvBridge()
        self.sub_img_ = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height) /
                            2+(descentre+rows_to_watch)][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        moments_dict = cv2.moments(mask, False)
        if abs(moments_dict['m00']) > 0.0:
            cx, cy = moments_dict['m10'] / \
                moments_dict['m00'], moments_dict['m01']/moments_dict['m00']
        else:
            cy, cx = height/2, width/2

        # Bitwise-AND mask and original image
        binary_image = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Draw the centroid in the resultut image
        cv2.circle(binary_image, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", binary_image)

        cv2.waitKey(1)

        error_x = cx - width / 2
        twist_object = Twist()
        twist_object.linear.x = 0.2
        twist_object.angular.z = -error_x / 100
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        self.cmd_vel_pub.publish(twist_object)

    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()

    def spin(self):
        rate = rospy.Rate(5)
        while not self.ctrl_c:
            rate.sleep()

    def shutdownhook(self):
        self.clean_up()
        rospy.loginfo("shutdown time!")
        self.ctrl_c = True


def main():
    rospy.init_node('line_following_node', anonymous=True)

    line_follower_object = LineFollower()
    line_follower_object.spin()


if __name__ == '__main__':
    main()
