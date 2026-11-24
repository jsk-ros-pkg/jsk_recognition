#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv_bridge
import rospy
from sensor_msgs.msg import Image
from jsk_topic_tools import ConnectionBasedTransport


class SplitImage(ConnectionBasedTransport):

    def __init__(self):
        super(SplitImage, self).__init__()
        self.vertical_parts = rospy.get_param('~vertical_parts', 1)
        self.horizontal_parts = rospy.get_param('~horizontal_parts', 1)
        self.pubs = []
        for v in range(self.vertical_parts):
            pubs = []
            for h in range(self.horizontal_parts):
                pubs.append(
                    self.advertise(
                        '~output/vertical{0:02}/horizontal{1:02}'.format(v, h),
                        Image,
                        queue_size=10))
            self.pubs.append(pubs)
        self.bridge = cv_bridge.CvBridge()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._split_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _split_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = img.shape
        for v in range(self.vertical_parts):
            for h in range(self.horizontal_parts):
                v_pixels = float(height) / self.vertical_parts
                h_pixels = float(width) / self.horizontal_parts
                split_img = img[int(v*v_pixels):int((v+1)*v_pixels),
                                int(h*h_pixels):int((h+1)*h_pixels)]
                pub_msg = self.bridge.cv2_to_imgmsg(split_img, encoding='bgr8')
                pub_msg.header = msg.header
                self.pubs[v][h].publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('split_image')
    rospy.logwarn("split_image.py would be deprecated. Please use NODELET version of split_image")
    SplitImage()
    rospy.spin()
