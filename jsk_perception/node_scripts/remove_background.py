#!/usr/bin/env python

from __future__ import print_function

import os
import sys

from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
from PIL import Image
import rospy
import sensor_msgs.msg
from jsk_recognition_utils import cv_bridge

import onnxruntime as ort

from rembg_libs.import_rembg import rembg
from rembg.bg import remove



class RemoveBackground(ConnectionBasedTransport):

    def __init__(self):
        super(RemoveBackground, self).__init__()
        pretrained_model = rospy.get_param('~pretrained_model')
        self.session = ort.InferenceSession(
            pretrained_model, providers=ort.get_available_providers())
        self.pub = self.advertise(
            '~output', sensor_msgs.msg.Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', sensor_msgs.msg.Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')
        pil_img = Image.fromarray(img)
        non_bg_img = remove(pil_img, only_mask=True,
                            session=self.session)
        img = np.array(non_bg_img)
        if self.pub.get_num_connections() > 0:
            msg_viz = bridge.cv2_to_imgmsg(img, encoding='mono8')
            msg_viz.header = imgmsg.header
            self.pub.publish(msg_viz)


if __name__ == '__main__':
    rospy.init_node('remove_background')
    node = RemoveBackground()
    rospy.spin()
