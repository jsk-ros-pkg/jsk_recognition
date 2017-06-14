#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from unittest import TestCase

from jsk_recognition_msgs.msg import PolygonArray


class TestPolygonArrayLikelihoodFilter(TestCase):
    def polygon_cb(self, msg):
        self.msg = msg

    def test_filter(self):
        self.msg = None
        self.sub_polygon = rospy.Subscriber("/polygon_array_likelihood_filter/output_polygons",
                                            PolygonArray, self.polygon_cb)

        for i in range(30):
            if self.msg is not None:
                self.sub_polygon.unregister()
                break
            rospy.sleep(1.0)
        self.assertIsNotNone(self.msg, "No message received for 30 seconds")

        self.assertEqual(len(self.msg.polygons), 2,
                         "polygons that have likelihood more than 0.8 are 2")
        for l in self.msg.likelihood:
            self.assertGreaterEqual(l, 0.8, "likelihood is greater equal 0.8")


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_polygon_array_likelihood_filter")
    rostest.rosrun("jsk_pcl_ros_utils", "polygon_array_likelihood_filter", TestPolygonArrayLikelihoodFilter)
