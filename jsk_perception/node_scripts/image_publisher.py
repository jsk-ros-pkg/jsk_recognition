#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

import cv2
import numpy as np

import cv_bridge
from cv_bridge.boost.cv_bridge_boost import getCvType
import dynamic_reconfigure.server
import rospy

from jsk_perception.cfg import ImagePublisherConfig
from jsk_topic_tools import jsk_logerr
from jsk_topic_tools import jsk_logwarn
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class ImagePublisher(object):

    def __init__(self):
        dynamic_reconfigure.server.Server(
            ImagePublisherConfig, self._cb_dyn_reconfig)
        self.encoding = rospy.get_param('~encoding', 'bgr8')
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.pub = rospy.Publisher('~output', Image, queue_size=1)
        self.publish_info = rospy.get_param('~publish_info', True)
        if self.publish_info:
            self.pub_info = rospy.Publisher(
                '~output/camera_info', CameraInfo, queue_size=1)

    def _cb_dyn_reconfig(self, config, level):
        self.file_name = config['file_name']
        config['file_name'] = os.path.abspath(self.file_name)
        return config

    def publish(self):
        now = rospy.Time.now()
        bridge = cv_bridge.CvBridge()
        img_bgr = cv2.imread(self.file_name)
        if img_bgr is None:
            jsk_logwarn('cannot read the image at {}'
                        .format(self.file_name))
            return
        # resolve encoding
        encoding = rospy.get_param('~encoding', 'bgr8')
        if getCvType(encoding) == 0:
            # mono8
            img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        elif getCvType(encoding) == 2:
            # 16UC1
            img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            img = img.astype(np.float32)
            img = img / 255 * (2 ** 16)
            img = img.astype(np.uint16)
        elif getCvType(encoding) == 5:
            # 32FC1
            img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            img = img.astype(np.float32)
            img /= 255
        elif getCvType(encoding) == 16:
            # 8UC3
            if encoding in ('rgb8', 'rgb16'):
                # RGB
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            else:
                img = img_bgr
        else:
            jsk_logerr('unsupported encoding: {0}'.format(encoding))
            return
        # setup ros message and publish
        imgmsg = bridge.cv2_to_imgmsg(img, encoding=encoding)
        imgmsg.header.stamp = now
        imgmsg.header.frame_id = self.frame_id
        self.pub.publish(imgmsg)
        if self.publish_info:
            info = CameraInfo()
            info.header.stamp = now
            info.header.frame_id = self.frame_id
            info.width = imgmsg.width
            info.height = imgmsg.height
            self.pub_info.publish(info)


if __name__ == '__main__':
    rospy.init_node('image_publisher')
    rate = rospy.Rate(rospy.get_param('rate', 1))
    img_pub = ImagePublisher()
    while not rospy.is_shutdown():
        img_pub.publish()
        rate.sleep()
