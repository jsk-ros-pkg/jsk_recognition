#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image
import cv_bridge


class MaskImageToLabel(ConnectionBasedTransport):
    def __init__(self):
        super(MaskImageToLabel, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self._sub = rospy.Subscriber('~input', Image, self._apply)

    def unsubscribe(self):
        self._sub.unregister()

    def _apply(self, msg):
        bridge = cv_bridge.CvBridge()
        mask = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if mask.size == 0:
            rospy.logdebug('Skipping empty image')
            return
        label = np.zeros(mask.shape, dtype=np.int32)
        label[mask == 0] = 0
        label[mask == 255] = 1
        label_msg = bridge.cv2_to_imgmsg(label, encoding='32SC1')
        label_msg.header = msg.header
        self._pub.publish(label_msg)


if __name__ == '__main__':
    rospy.init_node('mask_image_to_label')
    mask2label = MaskImageToLabel()
    rospy.spin()
