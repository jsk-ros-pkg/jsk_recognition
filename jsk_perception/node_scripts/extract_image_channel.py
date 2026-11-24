#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import re
import sys

import numpy as np

import cv_bridge
import dynamic_reconfigure.server
from jsk_perception.cfg import ExtractImageChannelConfig
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logwarn_throttle
import rospy
from sensor_msgs.msg import Image


# See https://github.com/ros-perception/vision_opencv/pull/141
# for PR to upstream
class CvBridgeArbitraryChannels(cv_bridge.CvBridge):

    def encoding_to_dtype_with_channels(self, encoding):
        try:
            return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))
        except cv_bridge.CvBridgeError as e:
            try:
                vals = re.split('(.+)C(.+)', encoding)
                dtype, n_channels = self.numpy_type_to_cvtype[vals[1]], int(vals[2])
                return dtype, n_channels
            except Exception as e:
                raise cv_bridge.CvBridgeError(e)


class ExtractImageChannel(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        dynamic_reconfigure.server.Server(
            ExtractImageChannelConfig, self._config_callback)
        self.pub = self.advertise('~output', Image, queue_size=1)

    def _config_callback(self, config, level):
        self.channel = config.channel
        return config

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._callback)

    def unsubscribe(self):
        self.sub.unregister()

    def _callback(self, imgmsg):
        if self.channel < 0:
            return
        bridge = CvBridgeArbitraryChannels()
        img = bridge.imgmsg_to_cv2(imgmsg)
        if img.ndim == 2:
            img = img[:, :, np.newaxis]
        if not (self.channel < img.shape[2]):
            logwarn_throttle(10,
                'Invalid channel {} is specified for image with {} channels'
                .format(self.channel, img.shape[2]))
            return
        out_img = np.zeros(img.shape[:2], dtype=img.dtype)
        out_img = img[:, :, self.channel]
        out_imgmsg = bridge.cv2_to_imgmsg(out_img)
        out_imgmsg.header = imgmsg.header
        self.pub.publish(out_imgmsg)


if __name__ == '__main__':
    rospy.init_node('extract_image_channel')
    ExtractImageChannel()
    rospy.spin()
