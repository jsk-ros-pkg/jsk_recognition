#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image


class ImageToLabel(ConnectionBasedTransport):
    def __init__(self):
        super(ImageToLabel, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self._sub = rospy.Subscriber('~input', Image, self._convert)

    def unsubscribe(self):
        self._sub.unregister()

    def _convert(self, msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        label = np.ones(img.shape[:2], dtype=np.int32)
        label_msg = bridge.cv2_to_imgmsg(label, encoding='32SC1')
        label_msg.header = msg.header
        self._pub.publish(label_msg)


if __name__ == '__main__':
    rospy.init_node('image_to_label')
    img2label = ImageToLabel()
    rospy.spin()
