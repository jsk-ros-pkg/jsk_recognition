#!/usr/bin/env python
import os

import cv2

import rospy
import cv_bridge
import dynamic_reconfigure.server

from jsk_topic_tools import jsk_logwarn
from jsk_perception.cfg import ImagePublisherConfig
from sensor_msgs.msg import Image, CameraInfo


class ImagePublisher(object):

    def __init__(self):
        dynamic_reconfigure.server.Server(
            ImagePublisherConfig, self._cb_dyn_reconfig)
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
        img = cv2.imread(self.file_name)
        if img is None:
            jsk_logwarn('cannot read the image at {}'
                        .format(self.file_name))
            return
        imgmsg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
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
