#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import cv2
import cv_bridge
import jsk_recognition_utils
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image
import pkg_resources
from distutils.version import StrictVersion


class TileImages(ConnectionBasedTransport):
    def __init__(self):
        super(TileImages, self).__init__()
        self.input_topics = rospy.get_param('~input_topics', [])
        if not self.input_topics:
            rospy.logerr('need to specify input_topics')
            sys.exit(1)
        self.draw_topic_name = rospy.get_param('~draw_topic_name', False)
        self.approximate_sync = rospy.get_param('~approximate_sync', True)
        if (StrictVersion(pkg_resources.get_distribution('message_filters').version) < StrictVersion('1.11.4') and
            self.approximate_sync):
            rospy.logerr('hydro message_filters does not support approximate sync. Force to set ~approximate_sync=false')
            self.approximate_sync = False
        self.pub_img = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub_img_list = []
        for i, input_topic in enumerate(self.input_topics):
            sub_img = message_filters.Subscriber(input_topic, Image)
            self.sub_img_list.append(sub_img)
        if self.approximate_sync:
            async = message_filters.ApproximateTimeSynchronizer(
                self.sub_img_list, queue_size=10, slop=1)
            async.registerCallback(self._apply)
        else:
            sync = message_filters.TimeSynchronizer(
                self.sub_img_list, queue_size=10)
            sync.registerCallback(self._apply)
    def unsubscribe(self):
        for sub in self.sub_img_list:
            sub.sub.unregister()

    def _apply(self, *msgs):
        bridge = cv_bridge.CvBridge()
        imgs = []
        for msg, topic in zip(msgs, self.input_topics):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.draw_topic_name:
                cv2.putText(img, topic, (0, 50), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 4)
            imgs.append(img)
        out_bgr = jsk_recognition_utils.get_tile_image(imgs)
        imgmsg = bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
        self.pub_img.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('tile_image')
    tile_image = TileImages()
    rospy.spin()
