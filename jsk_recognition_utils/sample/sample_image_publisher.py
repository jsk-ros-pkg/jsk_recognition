#!/usr/bin/env python

from __future__ import division

import os.path as osp

import cv2
import cv_bridge
import genpy
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import yaml


class SampleImagePublisher(object):

    def __init__(self):
        self.pub_img = rospy.Publisher(
            '~output/image_raw', Image, queue_size=1)
        self.pub_info = rospy.Publisher(
            '~output/camera_info', CameraInfo, queue_size=1)

        img_file = osp.expanduser(rospy.get_param('~image_file'))
        info_file = osp.expanduser(rospy.get_param('~camera_info_file'))
        encoding = rospy.get_param('~encoding', 'passthrough')
        frame_id = rospy.get_param('~frame_id', 'camera_rgb_optical_frame')
        rate = rospy.get_param('~rate', 1.0)

        # read image from file
        if img_file.endswith('.jpg') or img_file.endswith('.png'):
            img = cv2.imread(img_file, cv2.IMREAD_UNCHANGED)
        elif img_file.endswith('.npz'):
            img = np.load(img_file)['arr_0']
        else:
            rospy.logerr('Unsupported file format: {}'.format(img_file))
        bridge = cv_bridge.CvBridge()
        self.imgmsg = bridge.cv2_to_imgmsg(img, encoding)
        self.imgmsg.header.frame_id = frame_id

        # read camera_info from file
        with open(info_file, 'r') as f:
            info = yaml.load(f)
        self.infomsg = CameraInfo()
        genpy.message.fill_message_args(self.infomsg, info)

        rospy.Timer(rospy.Duration(1 / rate), self._cb)

    def _cb(self, event):
        self.imgmsg.header.stamp = event.current_real
        self.infomsg.header.stamp = event.current_real
        self.pub_img.publish(self.imgmsg)
        self.pub_info.publish(self.infomsg)


if __name__ == '__main__':
    rospy.init_node('sample_image_publisher')
    app = SampleImagePublisher()
    rospy.spin()
