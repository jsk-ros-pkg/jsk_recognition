#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv_bridge
from jsk_recognition_utils import bounding_rect_of_mask
from jsk_recognition_utils import get_tile_image
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools import jsk_loginfo
from jsk_topic_tools import warn_no_remap
import message_filters
import rospy
from sensor_msgs.msg import Image


class LabelImageDecomposer(ConnectionBasedTransport):

    def __init__(self):
        super(LabelImageDecomposer, self).__init__()
        self.pub_img = self.advertise('~output', Image, queue_size=5)
        self._publish_tile = rospy.get_param('~publish_tile', False)
        jsk_loginfo('~publish_info: {}'.format(self._publish_tile))
        if self._publish_tile:
            self.pub_tile = self.advertise('~output/tile', Image, queue_size=5)

    def subscribe(self):
        self.sub_img = message_filters.Subscriber('~input', Image)
        self.sub_label = message_filters.Subscriber('~input/label', Image)
        warn_no_remap('~input', '~input/label')
        use_async = rospy.get_param('~approximate_sync', False)
        jsk_loginfo('~approximate_sync: {}'.format(use_async))
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            jsk_loginfo('~slop: {}'.format(slop))
            async = message_filters.ApproximateTimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=10, slop=slop)
            async.registerCallback(self._apply)
            if self._publish_tile:
                async.registerCallback(self._apply_tile)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=10)
            sync.registerCallback(self._apply)
            if self._publish_tile:
                sync.registerCallback(self._apply_tile)

    def unsubscribe(self):
        self.sub_img.sub.unregister()
        self.sub_label.sub.unregister()

    def _apply(self, img_msg, label_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        label_img = bridge.imgmsg_to_cv2(label_msg)

        applied = img.copy()
        applied[label_img == 0] = 0
        applied_msg = bridge.cv2_to_imgmsg(applied, encoding='bgr8')
        applied_msg.header = img_msg.header
        self.pub_img.publish(applied_msg)

    def _apply_tile(self, img_msg, label_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        label_img = bridge.imgmsg_to_cv2(label_msg)

        imgs = []
        labels = np.unique(label_img)
        for label in labels:
            if label == 0:
                # should be skipped 0, because
                # 0 is to label image as black region to mask image
                continue
            img_tmp = img.copy()
            mask = label_img == label
            img_tmp[~mask] = 0
            img_tmp = bounding_rect_of_mask(img_tmp, mask)
            imgs.append(img_tmp)
        tile_img = get_tile_image(imgs)
        tile_msg = bridge.cv2_to_imgmsg(tile_img, encoding='bgr8')
        tile_msg.header = img_msg.header
        self.pub_tile.publish(tile_msg)


if __name__ == '__main__':
    rospy.init_node('label_image_decomposer')
    label_image_decomposer = LabelImageDecomposer()
    rospy.spin()
