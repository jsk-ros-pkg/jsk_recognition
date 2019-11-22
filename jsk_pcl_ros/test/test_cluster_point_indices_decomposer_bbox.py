#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import unittest

from nose.tools import assert_false
from nose.tools import assert_true

from jsk_recognition_msgs.msg import BoundingBoxArray
import rospy
import rostest


PKG = 'jsk_pcl_ros'
NAME = 'test_cluster_point_indices_decomposer_box'


class TestClusterPointIndicesDecomposerBbox(unittest.TestCase):
    def __init__(self, *args):
        super(TestClusterPointIndicesDecomposerBbox, self).__init__(*args)
        rospy.init_node(NAME)
        self.check_times = rospy.get_param('~check_times', 1)

    def test_bbox(self):
        is_boxes_empty = True
        for i in range(self.check_times):
            msg = rospy.wait_for_message("~boxes", BoundingBoxArray)
            for box in msg.boxes:
                is_boxes_empty = False
                assert_false(math.isnan(box.pose.position.x),
                             'pose.position.x is nan')
                assert_false(math.isnan(box.pose.position.y),
                             'pose.position.y is nan')
                assert_false(math.isnan(box.pose.position.z),
                             'pose.position.z is nan')
                assert_false(math.isnan(box.pose.orientation.x),
                             'pose.orientation.x is nan')
                assert_false(math.isnan(box.pose.orientation.y),
                             'pose.orientation.y is nan')
                assert_false(math.isnan(box.pose.orientation.z),
                             'pose.orientation.z is nan')
                assert_false(math.isnan(box.pose.orientation.w),
                             'pose.orientation.w is nan')
                assert_false(math.isnan(box.dimensions.x),
                             'dimensions.x is nan')
                assert_false(math.isnan(box.dimensions.y),
                             'dimensions.y is nan')
                assert_false(math.isnan(box.dimensions.z),
                             'dimensions.z is nan')
                assert_true(box.dimensions.x != 0, 'dimensions.x is 0')
                assert_true(box.dimensions.y != 0, 'dimensions.y is 0')
                assert_true(box.dimensions.z != 0, 'dimensions.z is 0')
        assert_true(
            not is_boxes_empty,
            'Bboxes array is always empty in %d trials' % self.check_times
        )


if __name__ == '__main__':
    rostest.run(PKG, NAME, TestClusterPointIndicesDecomposerBbox, sys.argv)
