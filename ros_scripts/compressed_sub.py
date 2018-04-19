#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs
CompressedImage. It converts the CompressedImage into a numpy.ndarray,
then detects and marks features in that image. It finally displays
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import cv_bridge

# Ros libraries
import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

import message_filters
from Auto_Detection.depth_detection import detection_with_depth
from Auto_Detection.max_version import armour_detection

VERBOSE=False
speed = 1.0

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size = 1)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # subscribed Topic
        # self.subscriber = rospy.Subscriber("/camera/image/compressed",
        #     CompressedImage, self.callback,  queue_size = 1)

        # self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed",
        #     CompressedImage, self.callback, queue_size = 1)
        #
        # self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",
        #     Image, self.depth_callback, queue_size = 1)

        self.color_subscriber = message_filters.Subscriber("/camera/color/image_raw/compressed",
            CompressedImage)#, queue_size = 1)

        self.depth_subscriber = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",
            Image)#, queue_size = 1)

        self.ts = message_filters.TimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber], 50)
        self.ts.registerCallback(self.sync_callback)

        self.bridge = cv_bridge.CvBridge()


        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def sync_callback(self, color_data, depth_data):

        np_arr = np.fromstring(color_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        depth_frame = self.bridge.imgmsg_to_cv2(depth_data)

        x, y, r = detection_with_depth(image_np, depth_frame)
        # armour_detection(image_np)

        # cv2.imshow('color_img', image_np)
        # cv2.imshow('depth_img', depth_frame)
        # cv2.waitKey(2)


        imsize = image_np.shape
        # print imsize
        if x is not None and y is not None:
            print "x:", x-imsize[1]//2, "y:", y-imsize[0]//2, "r:", r, "distance:", depth_frame[y][x]
            dx = x-imsize[1]//2
            dd = depth_frame[y][x] - 400
            # self.move([((37-r)/37.0)*speed,0,speed*(dx/float(imsize[1]//2))])
            self.move([dd/1600.0*speed,0,speed*(dx/float(imsize[1]//2))])
        else:
            self.move([0,0,0])

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

    def move(self, vels):
        t = Twist()
        t.linear.x = vels[0]
        t.linear.y = vels[1]
        t.angular.z = vels[2] * 0.5
        self.twist_pub.publish(t)

    def callback(self, ros_data):
        # '''Callback function of subscribed topic.
        # Here images get converted and features detected'''
        # if VERBOSE :
        #     print 'received image of type: "%s"' % ros_data.format
        #
        # #### direct conversion to CV2 ####
        # # np_arr = np.fromstring(ros_data.data, np.uint8)
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        #
        #
        #
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
