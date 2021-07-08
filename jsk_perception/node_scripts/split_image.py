#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import cv_bridge
import rospy
import threading
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
        rate = rospy.get_param('~rate', 30.0)
        rospy.Timer(rospy.Duration(1.0 / rate), self._timer_cb)
        self.msg = None
        self.lock = threading.Lock()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._split_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _split_cb(self, msg):
        with self.lock:
            self.msg = msg

    def _timer_cb(self, event):
        with self.lock:
            if self.msg is None:
                return
            header = copy.deepcopy(self.msg.header)
            img = self.bridge.imgmsg_to_cv2(self.msg)
        height, width, _ = img.shape
        for v in range(self.vertical_parts):
            for h in range(self.horizontal_parts):
                v_pixels = float(height) / self.vertical_parts
                h_pixels = float(width) / self.horizontal_parts
                split_img = img[int(v*v_pixels):int((v+1)*v_pixels),
                                int(h*h_pixels):int((h+1)*h_pixels)]
                pub_msg = self.bridge.cv2_to_imgmsg(split_img,
                                                    encoding=self.msg.encoding)
                pub_msg.header = header
                self.pubs[v][h].publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('split_image')
    SplitImage()
    rospy.spin()
