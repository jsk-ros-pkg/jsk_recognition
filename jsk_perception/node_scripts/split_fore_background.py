#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv_bridge
from jsk_recognition_utils.depth import split_fore_background
from jsk_topic_tools import jsk_logwarn
from jsk_topic_tools import jsk_logerr
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image


class SplitForeBackground(ConnectionBasedTransport):

    def __init__(self):
        super(SplitForeBackground, self).__init__()
        self.fg_pub_ = self.advertise('~output/fg', Image, queue_size=10)
        self.fg_mask_pub_ = self.advertise(
            '~output/fg_mask', Image, queue_size=10)
        self.bg_pub_ = self.advertise('~output/bg', Image, queue_size=10)
        self.bg_mask_pub_ = self.advertise(
            '~output/bg_mask', Image, queue_size=10)

    def subscribe(self):
        self.sub_ = message_filters.Subscriber('~input', Image)
        self.sub_depth_ = message_filters.Subscriber('~input/depth', Image)
        if rospy.get_param('~approximate_sync', False):
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub_, self.sub_depth_], queue_size=10, slop=.1)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub_, self.sub_depth_], queue_size=10)
        sync.registerCallback(self._apply)

    def unsubscribe(self):
        self.sub_.sub.unregister()
        self.sub_depth_.sub.unregister()

    def _apply(self, img_msg, depth_msg):
        # validation
        supported_encodings = {'16UC1', '32FC1'}
        if depth_msg.encoding not in supported_encodings:
            jsk_logwarn('Unsupported depth image encoding: {0}'
                        .format(depth_msg.encoding))
        if not (img_msg.height == depth_msg.height and
                img_msg.width == depth_msg.width):
            jsk_logerr('size of raw and depth image is different')
            return
        # split fg/bg and get each mask
        bridge = cv_bridge.CvBridge()
        depth = bridge.imgmsg_to_cv2(depth_msg)
        if depth_msg.encoding == '32FC1':
            # convert float to int (handle 32FC1 as 16UC1)
            depth = depth.copy()
            depth[np.isnan(depth)] = 0
            depth *= 255
            depth = depth.astype(np.uint8)
        fg_mask, bg_mask = split_fore_background(depth)
        # publish cropped
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # fg
        fg = img.copy()
        fg[~fg_mask] = 0
        fg_msg = bridge.cv2_to_imgmsg(fg, encoding=img_msg.encoding)
        fg_msg.header = img_msg.header
        self.fg_pub_.publish(fg_msg)
        # bg
        bg = img.copy()
        bg[~bg_mask] = 0
        bg_msg = bridge.cv2_to_imgmsg(bg, encoding=img_msg.encoding)
        bg_msg.header = img_msg.header
        self.bg_pub_.publish(bg_msg)
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
