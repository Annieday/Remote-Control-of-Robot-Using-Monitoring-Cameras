#!/usr/bin/env python
import numpy as np
import cv2, cv_bridge

# Ros libraries
import roslib
import rospy

import sys, select
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image

# from Auto_Detection.depth_detection import detect_with_depth
from Auto_Detection.depth_detection import detection_with_depth

speed = 0.4
rspeed = 0.8

key_mapping = {
    'i': [speed, 0, 0],
    'k': [-speed, 0, 0],
    'j': [0, speed, 0],
    'l': [0, -speed, 0],
    'd': [0, 0, rspeed],
    'a': [0, 0, -rspeed]
}

class ArmorFollower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.depth_frame = None
        self.color_frame = None

        # published topic
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # subscribed topics
        self.depth_sub = rospy.Subscriber("/camera/color/image_raw/compressed",
            CompressedImage, self.color_callback,  queue_size=1)
        self.color_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",
            Image, self.depth_callback,  queue_size=1)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg)

        # cv2.imshow("depth", self.depth_frame)
        # cv2.waitKey(2)
        self.detection()

    def color_callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.color_frame = image_np

        # cv2.imshow("color", self.color_frame)
        # cv2.waitKey(2)
        self.detection()

    def detection(self):
        if self.color_frame is not None and self.depth_frame is not None:
            ### detection here ###
            pass
            # print "detection"
            detection_with_depth(self.color_frame, self.depth_frame)
            # cv2.imshow("depth", self.depth_frame)
            # cv2.imshow("color", self.color_frame)
            cv2.waitKey(5)

    def move(self, vels):
        t = Twist()
        t.linear.x = vels[0]
        t.linear.y = vels[1]
        t.angular.z = vels[2]
        twist_pub.publish(t)

def main():
    '''Initializes and cleanup ros node'''
    af = ArmorFollower()
    rospy.init_node('armor_follower_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Armor Follower module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
