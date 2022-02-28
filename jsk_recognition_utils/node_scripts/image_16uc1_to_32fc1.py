#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys

import numpy as np

print(sys.version)
# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '2':
    python3_path = '/opt/ros/{}/lib/python3/dist-packages/'.format(
        os.environ['ROS_DISTRO'])
    sys.path.insert(0, python3_path)
    import cv_bridge
    sys.path.remove(python3_path)
else:
    import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image


class Image16UC1To32FC1(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.transport)

    def unsubscribe(self):
        self.sub.unregister()

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
    rospy.init_node('image_16uc1_to_32fc1')
    Image16UC1To32FC1()
    rospy.spin()
