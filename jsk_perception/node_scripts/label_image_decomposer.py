#!/usr/bin/env python

import sys

import cv2
import matplotlib
matplotlib.use('Agg')  # NOQA
import matplotlib.cm
import numpy as np
import scipy.ndimage

import cv_bridge
import dynamic_reconfigure.server
from jsk_recognition_utils import bounding_rect_of_mask
from jsk_recognition_utils import get_tile_image
from jsk_recognition_utils.color import labelcolormap
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools import warn_no_remap
import message_filters
import rospy
from sensor_msgs.msg import Image

from jsk_perception.cfg import LabelImageDecomposerConfig


def get_text_color(color):
    if color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114 > 170:
        return (0, 0, 0)
    return (255, 255, 255)


def label2rgb(lbl, img=None, label_names=None, alpha=0.3, bg_label=0):
    if label_names is None:
        n_labels = lbl.max() + 1  # +1 for bg_label 0
    else:
        n_labels = len(label_names)
    cmap = labelcolormap(256)
    cmap = (cmap * 255).astype(np.uint8)
    bg_color, cmap = cmap[0], cmap[1:]  # bg_color is 0

    lbl_viz = np.zeros((lbl.shape[0], lbl.shape[1], 3), dtype=np.uint8)
    fg_mask = lbl != bg_label
    lbl_viz[fg_mask] = cmap[lbl[fg_mask] % 255]
    lbl_viz[~fg_mask] = bg_color

    if img is not None:
        if img.ndim == 3:
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        else:
            assert img.ndim == 2
            img_gray = img
        img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)
        lbl_viz = alpha * lbl_viz + (1 - alpha) * img_gray
        lbl_viz = lbl_viz.astype(np.uint8)

    if label_names is None:
        return lbl_viz

    np.random.seed(1234)
    labels = np.unique(lbl)
    labels = labels[labels != 0]
    for label in labels:
        mask = lbl == label
        mask = (mask * 255).astype(np.uint8)
        y, x = scipy.ndimage.center_of_mass(mask)
        y, x = map(int, [y, x])

        if lbl[y, x] != label:
            Y, X = np.where(mask)
            point_index = np.random.randint(0, len(Y))
            y, x = Y[point_index], X[point_index]

        text = label_names[label]
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        text_size, baseline = cv2.getTextSize(
            text, font_face, font_scale, thickness)

        color = get_text_color(lbl_viz[y, x])
        cv2.putText(lbl_viz, text,
                    (x - text_size[0] // 2, y),
                    font_face, font_scale, color, thickness)
    return lbl_viz


class LabelImageDecomposer(ConnectionBasedTransport):

    def __init__(self):
        super(LabelImageDecomposer, self).__init__()

        self._srv_dynparam = dynamic_reconfigure.server.Server(
            LabelImageDecomposerConfig, self._config_callback)

        self.pub_img = self.advertise('~output', Image, queue_size=5)
        self.pub_label_viz = self.advertise('~output/label_viz', Image,
                                            queue_size=5)
        self._bg_label = rospy.get_param('~bg_label', 0)  # ignored label
        self._only_label = rospy.get_param('~only_label', False)
        self._label_names = rospy.get_param('~label_names', None)
        # publish masks of fg/bg by decomposing each label
        self._publish_mask = rospy.get_param('~publish_mask', False)
        if self._publish_mask:
            self.pub_fg_mask = self.advertise('~output/fg_mask', Image,
                                              queue_size=5)
            self.pub_bg_mask = self.advertise('~output/bg_mask', Image,
                                              queue_size=5)
        # publish each region image. this can take time so optional.
        self._publish_tile = rospy.get_param('~publish_tile', False)
        rospy.loginfo('~publish_tile: {}'.format(self._publish_tile))
        if self._only_label and self._publish_tile:
            rospy.logerr('Can not publish tile image when ~only_label is true,'
                         ' so forcely set ~publish_tile to false.')
            self._publish_tile = False
        if self._publish_tile:
            self.pub_tile = self.advertise('~output/tile', Image, queue_size=5)

    def _config_callback(self, config, level):
        self._alpha = config.alpha
        return config

    def subscribe(self):
        self.sub_label = message_filters.Subscriber('~input/label', Image)
        if self._only_label:
            self.sub_label.registerCallback(self._apply)
            return
        self.sub_img = message_filters.Subscriber('~input', Image)
        warn_no_remap('~input', '~input/label')
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 10)
        rospy.loginfo('~approximate_sync: {}, queue_size: {}'
                      .format(use_async, queue_size))
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            rospy.loginfo('~slop: {}'.format(slop))
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub_label, self.sub_img],
                queue_size=queue_size, slop=slop)
            sync.registerCallback(self._apply)
            if self._publish_tile:
                sync.registerCallback(self._apply_tile)
        else:
            sync = message_filters.TimeSynchronizer(
                [self.sub_label, self.sub_img], queue_size=queue_size)
            sync.registerCallback(self._apply)
            if self._publish_tile:
                sync.registerCallback(self._apply_tile)

    def unsubscribe(self):
        self.sub_img.sub.unregister()
        self.sub_label.sub.unregister()

    def _apply(self, label_msg, img_msg=None):
        bridge = cv_bridge.CvBridge()
        label_img = bridge.imgmsg_to_cv2(label_msg)
        if img_msg:
            img = bridge.imgmsg_to_cv2(img_msg)
            # publish only valid label region
            applied = img.copy()
            applied[label_img == self._bg_label] = 0
            applied_msg = bridge.cv2_to_imgmsg(applied, encoding=img_msg.encoding)
            applied_msg.header = img_msg.header
            self.pub_img.publish(applied_msg)
            # publish visualized label
            if img_msg.encoding in {'16UC1', '32SC1'}:
                # do dynamic scaling to make it look nicely
                min_value, max_value = img.min(), img.max()
                img = (img - min_value) / (max_value - min_value) * 255
                img = img.astype(np.uint8)
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            img = None

        label_viz = label2rgb(label_img, img, label_names=self._label_names,
                              alpha=self._alpha, bg_label=self._bg_label)
        label_viz_msg = bridge.cv2_to_imgmsg(label_viz, encoding='rgb8')
        label_viz_msg.header = label_msg.header
        self.pub_label_viz.publish(label_viz_msg)

        # publish mask
        if self._publish_mask:
            bg_mask = (label_img == 0)
            fg_mask = ~bg_mask
            bg_mask = (bg_mask * 255).astype(np.uint8)
            fg_mask = (fg_mask * 255).astype(np.uint8)
            fg_mask_msg = bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
            fg_mask_msg.header = label_msg.header
            bg_mask_msg = bridge.cv2_to_imgmsg(bg_mask, encoding='mono8')
            bg_mask_msg.header = label_msg.header
            self.pub_fg_mask.publish(fg_mask_msg)
            self.pub_bg_mask.publish(bg_mask_msg)

    def _apply_tile(self, label_msg, img_msg):
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
