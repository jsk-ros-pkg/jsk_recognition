#!/usr/bin/env python

from __future__ import print_function

import os
import sys

from jsk_topic_tools import ConnectionBasedTransport
from dynamic_reconfigure.server import Server
import numpy as np
from PIL import Image
import rospy
import sensor_msgs.msg
from jsk_recognition_utils import cv_bridge
import onnxruntime as ort

from rembg_libs.import_rembg import rembg
from rembg.bg import remove

from jsk_perception_3rdparty.cfg import RemoveBackgroundConfig as Config


class RemoveBackground(ConnectionBasedTransport):

    def __init__(self):
        super(RemoveBackground, self).__init__()
        pretrained_model = rospy.get_param('~pretrained_model')
        self.srv = Server(Config, self.config_callback)
        self.session = ort.InferenceSession(
            pretrained_model, providers=ort.get_available_providers())
        self.pub = self.advertise(
            '~output', sensor_msgs.msg.Image, queue_size=1)
        self.pub_raw = self.advertise(
            '~output/raw', sensor_msgs.msg.Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', sensor_msgs.msg.Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def config_callback(self, config, level):
        self.mask_threshold = config.mask_threshold
        self.negative = config.negative
        return config

    def callback(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')
        pil_img = Image.fromarray(img)
        non_bg_img = remove(
            pil_img, only_mask=True,
            session=self.session)
        raw_mask_img = np.array(non_bg_img, dtype=np.uint8)
        img = np.array(255 * (np.array(non_bg_img) > self.mask_threshold),
                       dtype=np.uint8)
        if self.pub.get_num_connections() > 0:
            if self.negative is True:
                img = 255 - img
            msg_viz = bridge.cv2_to_imgmsg(img, encoding='mono8')
            msg_viz.header = imgmsg.header
            self.pub.publish(msg_viz)
        if self.pub_raw.get_num_connections() > 0:
            if self.negative is True:
                raw_mask_img = 255 - raw_mask_img
            msg_viz = bridge.cv2_to_imgmsg(raw_mask_img, encoding='mono8')
            msg_viz.header = imgmsg.header
            self.pub_raw.publish(msg_viz)


if __name__ == '__main__':
    rospy.init_node('remove_background')
    node = RemoveBackground()
    rospy.spin()
