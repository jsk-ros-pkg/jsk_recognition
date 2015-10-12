#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import cv_bridge
import jsk_recognition_utils
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image


class TileImages(ConnectionBasedTransport):
    def __init__(self):
        super(TileImages, self).__init__()
        self.input_topics = rospy.get_param('~input_topics', [])
        if not self.input_topics:
            rospy.logerr('need to specify input_topics')
            sys.exit(1)
        self.pub_img = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub_img_list = []
        for i, input_topic in enumerate(self.input_topics):
            sub_img = message_filters.Subscriber(input_topic, Image)
            self.sub_img_list.append(sub_img)
        async = message_filters.ApproximateTimeSynchronizer(
            self.sub_img_list, queue_size=10, slop=1)
        async.registerCallback(self._apply)

    def unsubscribe(self):
        for sub in self.sub_img_list:
            sub.sub.unregister()

    def _apply(self, *msgs):
        bridge = cv_bridge.CvBridge()
        imgs = []
        for msg in msgs:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            imgs.append(img)
        out_bgr = jsk_recognition_utils.get_tile_image(imgs)
        imgmsg = bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
        self.pub_img.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('tile_image')
    tile_image = TileImages()
    rospy.spin()
