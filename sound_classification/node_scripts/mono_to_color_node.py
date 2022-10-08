#!/usr/bin/env python

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image

from sound_classification.process_gray_image import img_jet


class MonoToColor(ConnectionBasedTransport):

    def __init__(self):
        super(MonoToColor, self).__init__()
        self.pub = self.advertise(
            '~output', Image, queue_size=1)

    def subscribe(self):
        sub = rospy.Subscriber(
            '~input', Image, self._convert, callback_args=None,
            queue_size=1, buff_size=2**24)
        self.subs = [sub]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _convert(self, img_msg):
        bridge = cv_bridge.CvBridge()
        mono = bridge.imgmsg_to_cv2(img_msg)
        bgr = img_jet(mono)
        out_img_msg = bridge.cv2_to_imgmsg(
            bgr.astype(np.uint8),
            encoding='bgr8')
        out_img_msg.header = img_msg.header
        self.pub.publish(out_img_msg)


if __name__ == '__main__':
    rospy.init_node('mono_to_color')
    app = MonoToColor()
    rospy.spin()
