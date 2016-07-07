#!/usr/bin/env python

import unittest

import numpy as np

import cv_bridge
from pcl_msgs.msg import PointIndices
import rospy
import rostest
from sensor_msgs.msg import Image


class TestPointIndicesToMaskImage(unittest.TestCase):

    def setUp(self):
        self.msg = {}
        self.pub_indices = rospy.Publisher(
            '~output/indices', PointIndices, queue_size=1)
        self.pub_img = rospy.Publisher('~output/image', Image, queue_size=1)
        self.sub_mask1 = rospy.Subscriber(
            '~input/mask1', Image, self.mask_cb, callback_args='mask1')
        self.sub_mask2 = rospy.Subscriber(
            '~input/mask2', Image, self.mask_cb, callback_args='mask2')

    def mask_cb(self, msg, name):
        self.msg[name] = msg

    def test_conversion(self):
        for name in ['mask1', 'mask2']:
            self._test_each_conversion(name)

    def _test_each_conversion(self, name):
        bridge = cv_bridge.CvBridge()

        imgmsg = bridge.cv2_to_imgmsg(np.zeros((30, 30), dtype=np.uint8),
                                      encoding='mono8')
        indices_msg = PointIndices()
        indices_msg.indices = np.arange(10, 30)
        while self.msg.get(name) is None:
            imgmsg.header.stamp = indices_msg.header.stamp = rospy.Time.now()
            self.pub_indices.publish(indices_msg)
            self.pub_img.publish(imgmsg)
            rospy.sleep(0.1)

        mask_in = bridge.imgmsg_to_cv2(
            self.msg[name], desired_encoding='mono8')
        mask_in = np.squeeze(mask_in)
        mask_expected = np.zeros((imgmsg.height, imgmsg.width), dtype=np.uint8)
        mask_expected.ravel()[np.arange(10, 30)] = 255
        np.testing.assert_array_equal(mask_in, mask_expected)


if __name__ == "__main__":
    PKG = 'jsk_pcl_ros_utils'
    ID = 'test_point_indices_to_mask_image'
    rospy.init_node(ID)
    rostest.rosrun(PKG, ID, TestPointIndicesToMaskImage)
