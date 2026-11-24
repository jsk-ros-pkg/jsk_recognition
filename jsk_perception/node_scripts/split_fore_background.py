#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv_bridge
from jsk_recognition_utils.depth import split_fore_background
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image


class SplitForeBackground(ConnectionBasedTransport):

    def __init__(self):
        super(SplitForeBackground, self).__init__()
        self.fg_mask_pub_ = self.advertise(
            '~output/fg_mask', Image, queue_size=10)
        self.bg_mask_pub_ = self.advertise(
            '~output/bg_mask', Image, queue_size=10)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._apply)

    def unsubscribe(self):
        self.sub.unregister()

    def _apply(self, depth_msg):
        # validation
        supported_encodings = {'16UC1', '32FC1'}
        if depth_msg.encoding not in supported_encodings:
            rospy.logwarn('Unsupported depth image encoding: {0}'
                          .format(depth_msg.encoding))
        # split fg/bg and get each mask
        bridge = cv_bridge.CvBridge()
        depth = bridge.imgmsg_to_cv2(depth_msg)
        nan_mask = None
        if depth_msg.encoding == '32FC1':
            # convert float to int (handle 32FC1 as 16UC1)
            nan_mask = np.isnan(depth)
            depth[nan_mask] = 0
            depth *= 255
            depth = depth.astype(np.uint8)
        fg_mask, bg_mask = split_fore_background(depth)
        if nan_mask is not None:
            fg_mask[nan_mask] = 0
            bg_mask[nan_mask] = 0
        # fg_mask
        fg_mask = (fg_mask * 255).astype(np.uint8)
        fg_mask_msg = bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
        fg_mask_msg.header = depth_msg.header
        self.fg_mask_pub_.publish(fg_mask_msg)
        # bg_mask
        bg_mask = (bg_mask * 255).astype(np.uint8)
        bg_mask_msg = bridge.cv2_to_imgmsg(bg_mask, encoding='mono8')
        bg_mask_msg.header = depth_msg.header
        self.bg_mask_pub_.publish(bg_mask_msg)


if __name__ == '__main__':
    rospy.init_node('split_fore_background')
    split_fbg = SplitForeBackground()
    rospy.spin()
