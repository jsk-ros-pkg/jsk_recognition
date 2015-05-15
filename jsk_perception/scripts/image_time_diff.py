#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import Cue, ImageDifferenceValue


class ImageTimeDiff(object):
    """Publish difference between stored image and current input one

    Publications:
        * ~output/diff: diff per pixel
        * ~output/diff_image: diff image

    Subscriptions:
        * ~input: input raw image
        * ~start: msg to store image and start comparing
        * ~stop: msg to relase image and stop comparing

    """
    def __init__(self):
        rospy.Subscriber('~input', Image, self._cb_image)
        rospy.Subscriber('~start', Cue, self._cb_start)
        rospy.Subscriber('~stop', Cue, self._cb_stop)
        self.pub_stored = None
        self.imgmsg = None

    def _cb_image(self, imgmsg):
        self.imgmsg = imgmsg

    def _cb_start(self, cue):
        while cue.header.stamp > self.imgmsg.header.stamp:
            rospy.sleep(0.1)
        imgmsg = self.imgmsg
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        pub_diff = rospy.Publisher('~output/diff',
                                   ImageDifferenceValue,
                                   queue_size=1)
        pub_diff_img = rospy.Publisher('~output/diff_image',
                                       Image,
                                       queue_size=1)
        pub_debug = rospy.Publisher('~debug', Image, queue_size=1)
        self.pub_stored = img, pub_diff, pub_diff_img, pub_debug

    def _cb_stop(self, cue):
        while cue.header.stamp > self.imgmsg.header.stamp:
            rospy.sleep(0.1)
        self.pub_stored = None

    def spin_once(self):
        if (self.imgmsg is None) or (self.pub_stored is None):
            return
        imgmsg = self.imgmsg
        pub_stored = self.pub_stored
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        img_stored, pub_diff, pub_diff_img, pub_debug = pub_stored
        # compute diff and publish the result
        diff_img = cv2.absdiff(img, img_stored)
        diff_msg = ImageDifferenceValue()
        diff_msg.header.stamp = imgmsg.header.stamp
        diff_msg.difference = diff_img.sum() / diff_img.size / 255.
        pub_diff.publish(diff_msg)
        pub_diff_img.publish(bridge.cv2_to_imgmsg(diff_img))
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
