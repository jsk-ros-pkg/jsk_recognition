#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ImageTimeDiff(object):
    """Publish difference between stored image and current input one

    Publications:
        * ~output/{namespace}: diff between stored image and current input

    Subscriptions:
        * ~input: input raw image
        * ~start_comparing: msg to store image and start comparing
        * ~stop_comparing: msg to relase image and stop comparing

    """
    def __init__(self):
        rospy.Subscriber('~input', Image, self._cb_image)
        rospy.Subscriber('~start_comparing', String, self._cb_start)
        rospy.Subscriber('~stop_comparing', String, self._cb_stop)
        self.pub_stored = {}
        self.imgmsg = None

    def _cb_image(self, msg):
        self.imgmsg = msg

    def _cb_start(self, msg):
        stamp = rospy.Time.now()
        namespace = msg.data
        pub = rospy.Publisher('~output/{0}'.format(namespace),
                              Image,
                              queue_size=1)
        pub_debug = rospy.Publisher('~debug/{0}'.format(namespace),
                                    Image, queue_size=1)
        while self.imgmsg is None or self.imgmsg.header.stamp < stamp:
            # wait for image at the same time
            rospy.sleep(0.1)
        imgmsg = self.imgmsg
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        self.pub_stored[namespace] = pub, img, pub_debug

    def _cb_stop(self, msg):
        namespace = msg.data
        del self.pub_stored[namespace]

    def spin_once(self):
        if self.imgmsg is None:
            return
        imgmsg = self.imgmsg
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        pub_stored = self.pub_stored
        for ns, (pub_diff, img_stored, pub_debug) in pub_stored.items():
            # compute diff for all publications
            diff = cv2.absdiff(img, img_stored)
            pub_diff.publish(bridge.cv2_to_imgmsg(diff))
            pub_debug.publish(bridge.cv2_to_imgmsg(img_stored))

    def spin(self):
        rate = rospy.Rate(rospy.get_param('rate', 10))
        while not rospy.is_shutdown():
            self.spin_once()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('image_time_diff')
    image_time_diff = ImageTimeDiff()
    image_time_diff.spin()
