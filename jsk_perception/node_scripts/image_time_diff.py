#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import cv2

import rospy
import cv_bridge
import message_filters
import dynamic_reconfigure.server

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import ImageDifferenceValue
from jsk_perception.cfg import ImageTimeDiffConfig


class ImageTimeDiff(object):
    """Publish difference between stored image and current input one

    Publications:
        * ~output/diff: diff per pixel: 0 - 1
        * ~output/diff_image: diff image

    Subscriptions:
        * ~input/hue: input hue image
        * ~input/saturation: input saturation image
        * ~start: msg to store image and start comparing
        * ~stop: msg to relase image and stop comparing

    """
    def __init__(self):
        dynamic_reconfigure.server.Server(ImageTimeDiffConfig,
                                          self._cb_dyn_reconfig)
        sub_hue = message_filters.Subscriber('~input/hue', Image)
        sub_saturation = message_filters.Subscriber('~input/saturation', Image)
        ts = message_filters.TimeSynchronizer([sub_hue, sub_saturation], 10)
        ts.registerCallback(self._cb_input)
        rospy.Subscriber('~start', Header, self._cb_start)
        rospy.Subscriber('~stop', Header, self._cb_stop)
        self.pub_stored = None
        self.input = None

    def _cb_dyn_reconfig(self, config, level):
        self.s_threshold = config['saturation_threshold']
        return config

    def _cb_input(self, imgmsg_hue, imgmsg_saturation):
        self.input = (imgmsg_hue, imgmsg_saturation)

    def _cb_start(self, header):
        while ( (self.input is None) or
                (header.stamp > self.input[0].header.stamp) ):
            rospy.sleep(0.1)
        imgmsg_hue, imgmsg_saturation = self.input
        bridge = cv_bridge.CvBridge()
        hue = bridge.imgmsg_to_cv2(imgmsg_hue, desired_encoding='mono8')
        saturation = bridge.imgmsg_to_cv2(imgmsg_saturation,
                                          desired_encoding='mono8')
        pub_diff = rospy.Publisher('~output/diff',
                                   ImageDifferenceValue,
                                   queue_size=1)
        pub_diff_img = rospy.Publisher('~output/diff_image',
                                       Image,
                                       queue_size=1)
        self.pub_stored = (hue, saturation, pub_diff, pub_diff_img)

    def _cb_stop(self, header):
        while header.stamp > self.input[0].header.stamp:
            rospy.sleep(0.1)
        self.pub_stored = None

    def spin_once(self):
        if (self.input is None) or (self.pub_stored is None):
            return
        imgmsg_hue, imgmsg_saturation = self.input
        pub_stored = self.pub_stored
        bridge = cv_bridge.CvBridge()
        hue = bridge.imgmsg_to_cv2(imgmsg_hue, desired_encoding='mono8')
        saturation = bridge.imgmsg_to_cv2(imgmsg_saturation,
                                          desired_encoding='mono8')
        hue_stored, saturation_stored, pub_diff, pub_diff_img = pub_stored
        # compute diff and publish the result
        diff_img = cv2.absdiff(hue, hue_stored)
        diff_msg = ImageDifferenceValue()
        diff_msg.header.stamp = imgmsg_hue.header.stamp
        mask = ( (saturation_stored > self.s_threshold) &
                 (saturation > self.s_threshold) )
        mask = mask.reshape((mask.shape[0], mask.shape[1]))
        filtered_diff_img = diff_img[mask]
        diff_msg.difference = ( filtered_diff_img.sum()
                                / filtered_diff_img.size
                                / 255. )
        pub_diff.publish(diff_msg)
        diff_img_msg = bridge.cv2_to_imgmsg(diff_img, encoding='mono8')
        diff_img_msg.header = diff_msg.header
        pub_diff_img.publish(diff_img_msg)

    def spin(self):
        rate = rospy.Rate(rospy.get_param('rate', 10))
        while not rospy.is_shutdown():
            self.spin_once()
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                pass


if __name__ == '__main__':
    rospy.init_node('image_time_diff')
    image_time_diff = ImageTimeDiff()
    image_time_diff.spin()
