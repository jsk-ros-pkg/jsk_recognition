#!/usr/bin/env python

# To avoid jsk_perception dependency, Copied from
# https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_perception/node_scripts/draw_classification_result.py
# https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_recognition_utils/python/jsk_recognition_utils/color.pyx

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cv2
import cv_bridge
from distutils.version import LooseVersion
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image


def bitget(byteval, idx):
    return ((byteval & (1 << idx)) != 0)


def labelcolormap(N=256):
    """Color map (RGB) considering label numbers N.
    Args:
        N (``int``): Number of labels.
    """
    cmap = np.zeros((N, 3),
                    dtype=np.float32)
    for i in range(0, N):
        id = i
        r, g, b = 0, 0, 0
        for j in range(0, 8):
            r = np.bitwise_or(r, (bitget(id, 0) << 7-j))
            g = np.bitwise_or(g, (bitget(id, 1) << 7-j))
            b = np.bitwise_or(b, (bitget(id, 2) << 7-j))
            id = (id >> 3)
        cmap[i, 0] = r / 255.
        cmap[i, 1] = g / 255.
        cmap[i, 2] = b / 255.
    return cmap


class DrawClassificationResult(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pub = self.advertise('~output', Image, queue_size=1)
        self.cmap = labelcolormap(255)

    def subscribe(self):
        self.sub = message_filters.Subscriber('~input', ClassificationResult)
        self.sub_img = message_filters.Subscriber('~input/image', Image)
        sync = message_filters.TimeSynchronizer(
            [self.sub, self.sub_img], queue_size=10)
        sync.registerCallback(self._draw)

    def unsubscribe(self):
        self.sub.unregister()
        self.sub_img.unregister()

    def _draw(self, cls_msg, imgmsg):
        bridge = cv_bridge.CvBridge()
        rgb = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')

        n_results = len(cls_msg.labels)
        for i in xrange(n_results):
            label = cls_msg.labels[i]
            color = self.cmap[label % len(self.cmap)] * 255
            legend_size = int(rgb.shape[0] * 0.1)
            rgb[:legend_size, :] = (np.array(color) * 255).astype(np.uint8)

            label_name = cls_msg.label_names[i]
            if len(label_name) > 16:
                label_name = label_name[:10] + '..' + label_name[-4:]
            label_proba = cls_msg.label_proba[i]
            title = '{0}: {1:.2%}'.format(label_name, label_proba)
            (text_w, text_h), baseline = cv2.getTextSize(
                title, cv2.FONT_HERSHEY_PLAIN, 1, 1)
            scale_h = legend_size / (text_h + baseline)
            scale_w = rgb.shape[1] / text_w
            scale = min(scale_h, scale_w)
            (text_w, text_h), baseline = cv2.getTextSize(
                label_name, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            if LooseVersion(cv2.__version__).version[0] < 3:
                line_type = cv2.CV_AA
            else:  # for opencv version > 3
                line_type = cv2.LINE_AA
            cv2.putText(rgb, title, (0, text_h - baseline),
                        cv2.FONT_HERSHEY_PLAIN + cv2.FONT_ITALIC,
                        scale, (255, 255, 255), 1,
                        line_type)

        out_msg = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        out_msg.header = imgmsg.header
        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('draw_classification_result')
    app = DrawClassificationResult()
    rospy.spin()
