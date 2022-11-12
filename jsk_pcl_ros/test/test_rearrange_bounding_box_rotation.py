#!/usr/bin/env python

from jsk_recognition_msgs.msg import BoundingBoxArray
import message_filters
from nose.tools import assert_false
from nose.tools import assert_true
import rospy
import sys
import unittest

PKG = 'jsk_pcl_ros'
NAME = 'test_rearrange_bounding_box_rotation'

class TestRearrangeBoundingBoxRotation(unittest.TestCase):
    def __init__(self, *args):
        super(TestRearrangeBoundingBoxRotation, self).__init__(*args)
        self.msg = None
        rospy.init_node(NAME)
        rotated_boxes_sub = rospy.Subscriber("/rearrange_bounding_box_check/output",
                                             BoundingBoxArray, self._cb)

    def test_rotation(self):
        while not rospy.is_shutdown():
            if self.msg:
                rospy.loginfo("Test rotated bounding box")
                assert_false((abs(self.msg.boxes[0].pose.orientation.w - 1.0) < 1e-7 and
                              abs(self.msg.boxes[0].pose.orientation.x) < 1e-7 and
                              abs(self.msg.boxes[0].pose.orientation.y) < 1e-7 and
                              abs(self.msg.boxes[0].pose.orientation.z) < 1e-7),
                             "The rotation calculations are not correct.")
                break

    def _cb(self, msg):
        self.msg = msg

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestRearrangeBoundingBoxRotation, sys.argv)
