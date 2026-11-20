#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

import numpy as np

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import Image

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


class Image16UC1To32FC1(ConnectionBasedTransport):

    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(self.__class__, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(self.__class__, self).__init__(
                'image_16uc1_to_32fc1', **kwargs)
        self.pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        if ROS_VERSION == 1:
            self.sub = rospy.Subscriber('~input', Image, self.transport)
        else:
            self.sub = ros.create_subscription(
                self, '~input', Image, self.transport)

    def unsubscribe(self):
        if ROS_VERSION == 1:
            self.sub.unregister()
        else:
            self.destroy_subscription(self.sub)

    def transport(self, msg):
        if msg.height * msg.width == 0:
            return
        bridge = cv_bridge.CvBridge()
        img_16uc1 = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        img_32fc1 = (img_16uc1.astype(np.float64) / (2**16)).astype(np.float32)
        out_msg = bridge.cv2_to_imgmsg(img_32fc1, '32FC1')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('image_16uc1_to_32fc1')
        Image16UC1To32FC1()
        rospy.spin()
    else:
        ros.rclpy.init()
        ros.spin(Image16UC1To32FC1())
