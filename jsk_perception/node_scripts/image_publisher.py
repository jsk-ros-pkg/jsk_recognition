#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from threading import Lock

import cv2
import numpy as np

import cv_bridge
from cv_bridge.boost.cv_bridge_boost import getCvType
import dynamic_reconfigure.server
import rospy

from jsk_perception.cfg import ImagePublisherConfig
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class ImagePublisher(object):

    def __init__(self):
        self.lock = Lock()
        self.imgmsg = None
        self.encoding = rospy.get_param('~encoding', 'bgr8')
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.fovx = rospy.get_param('~fovx', None)
        self.fovy = rospy.get_param('~fovy', None)
        if (self.fovx is None) != (self.fovy is None):
            rospy.logwarn('fovx and fovy should be specified, but '
                          'specified only {}'
                          .format('fovx' if self.fovx else 'fovy'))
        dynamic_reconfigure.server.Server(
            ImagePublisherConfig, self._cb_dyn_reconfig)
        self.pub = rospy.Publisher('~output', Image, queue_size=1)
        self.publish_info = rospy.get_param('~publish_info', True)
        if self.publish_info:
            self.pub_info = rospy.Publisher(
                '~output/camera_info', CameraInfo, queue_size=1)
        rate = rospy.get_param('~rate', 1.)
        rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def _cb_dyn_reconfig(self, config, level):
        file_name = config['file_name']
        config['file_name'] = os.path.abspath(file_name)
        img_bgr = cv2.imread(file_name)
        if img_bgr is None:
            rospy.logwarn('Could not read image file: {}'.format(file_name))
            with self.lock:
                self.imgmsg = None
        else:
            rospy.loginfo('Read the image file: {}'.format(file_name))
            with self.lock:
                self.imgmsg = self.cv2_to_imgmsg(img_bgr, self.encoding)
        return config

    def publish(self, event):
        if self.imgmsg is None:
            return
        now = rospy.Time.now()
        # setup ros message and publish
        with self.lock:
            self.imgmsg.header.stamp = now
            self.imgmsg.header.frame_id = self.frame_id
        self.pub.publish(self.imgmsg)
        if self.publish_info:
            info = CameraInfo()
            info.header.stamp = now
            info.header.frame_id = self.frame_id
            info.width = self.imgmsg.width
            info.height = self.imgmsg.height
            if self.fovx is not None and self.fovy is not None:
                fx = self.imgmsg.width / 2.0 / \
                    np.tan(np.deg2rad(self.fovx / 2.0))
                fy = self.imgmsg.height / 2.0 / \
                    np.tan(np.deg2rad(self.fovy / 2.0))
                cx = self.imgmsg.width / 2.0
                cy = self.imgmsg.height / 2.0
                info.K = np.array([fx, 0, cx,
                                   0, fy, cy,
                                   0, 0, 1.0])
                info.P = np.array([fx, 0, cx, 0,
                                   0, fy, cy, 0,
                                   0, 0, 1, 0])
                info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            self.pub_info.publish(info)

    def cv2_to_imgmsg(self, img_bgr, encoding):
        bridge = cv_bridge.CvBridge()
        # resolve encoding
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
            rospy.logerr('unsupported encoding: {0}'.format(encoding))
            return
        return bridge.cv2_to_imgmsg(img, encoding=encoding)


if __name__ == '__main__':
    rospy.init_node('image_publisher')
    ImagePublisher()
    rospy.spin()
