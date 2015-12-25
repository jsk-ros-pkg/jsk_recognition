#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from skimage.color import gray2rgb
from skimage.color import label2rgb
from skimage.segmentation import mark_boundaries

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
        self.pub_label_viz = self.advertise('~output/label_viz', Image,
                                            queue_size=5)
        # publish masks of fg/bg by decomposing each label
        self._publish_mask = rospy.get_param('~publish_mask', False)
        if self._publish_mask:
            self.pub_fg_mask = self.advertise('~output/fg_mask', Image,
                                              queue_size=5)
            self.pub_bg_mask = self.advertise('~output/bg_mask', Image,
                                                    queue_size=5)
        # publish each region image. this can take time so optional.
        self._publish_tile = rospy.get_param('~publish_tile', False)
        jsk_loginfo('~publish_tile: {}'.format(self._publish_tile))
        if self._publish_tile:
            self.pub_tile = self.advertise('~output/tile', Image, queue_size=5)

    def subscribe(self):
        self.sub_img = message_filters.Subscriber('~input', Image)
        self.sub_label = message_filters.Subscriber('~input/label', Image)
        warn_no_remap('~input', '~input/label')
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 10)
        jsk_loginfo('~approximate_sync: {}, queue_size: {}'
                    .format(use_async, queue_size))
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            jsk_loginfo('~slop: {}'.format(slop))
            async = message_filters.ApproximateTimeSynchronizer(
                [self.sub_img, self.sub_label],
                queue_size=queue_size, slop=slop)
            async.registerCallback(self._apply)
            if self._publish_tile:
                async.registerCallback(self._apply_tile)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=queue_size)
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
        # publish only valid label region
        applied = img.copy()
        applied[label_img == 0] = 0
        applied_msg = bridge.cv2_to_imgmsg(applied, encoding=img_msg.encoding)
        applied_msg.header = img_msg.header
        self.pub_img.publish(applied_msg)
        # publish visualized label
        if img_msg.encoding in {'16UC1', '32SC1'}:
            # do dynamic scaling to make it look nicely
            min_value, max_value = img.min(), img.max()
            img = (img - min_value) / (max_value - min_value) * 255
            img = gray2rgb(img)
        label_viz_img = label2rgb(label_img, img, bg_label=0)
        label_viz_img = mark_boundaries(label_viz_img, label_img, (1, 0, 0))
        label_viz_img = (label_viz_img * 255).astype(np.uint8)
        label_viz_msg = bridge.cv2_to_imgmsg(label_viz_img, encoding='rgb8')
        label_viz_msg.header = img_msg.header
        self.pub_label_viz.publish(label_viz_msg)
        # publish mask
        if self._publish_mask:
            bg_mask = (label_img == 0)
            fg_mask = ~bg_mask
            bg_mask = (bg_mask * 255).astype(np.uint8)
            fg_mask = (fg_mask * 255).astype(np.uint8)
            fg_mask_msg = bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
            fg_mask_msg.header = img_msg.header
            bg_mask_msg = bridge.cv2_to_imgmsg(bg_mask, encoding='mono8')
            bg_mask_msg.header = img_msg.header
            self.pub_fg_mask.publish(fg_mask_msg)
            self.pub_bg_mask.publish(bg_mask_msg)

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
