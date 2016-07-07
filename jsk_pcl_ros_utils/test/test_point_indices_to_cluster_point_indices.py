#!/usr/bin/env python

import unittest

import numpy as np

from jsk_recognition_msgs.msg import ClusterPointIndices
from pcl_msgs.msg import PointIndices
import rospy
import rostest


class TestPointIndicesToClusterPointIndices(unittest.TestCase):

    def setUp(self):
        self.msg = None
        self.indices = (0, 10, 20, 30)
        self.pub = rospy.Publisher('~output', PointIndices, queue_size=1)
        self.sub = rospy.Subscriber('~input', ClusterPointIndices, self.cb)

    def cb(self, msg):
        self.msg = msg

    def test_conversion(self):
        indices_msg = PointIndices()
        while self.msg is None:
            indices_msg.header.stamp = rospy.Time.now()
            indices_msg.indices = self.indices
            self.pub.publish(indices_msg)
            rospy.sleep(0.1)

        self.assertEqual(len(self.msg.cluster_indices), 1)
        self.assertTupleEqual(self.msg.cluster_indices[0].indices,
                              self.indices)


if __name__ == "__main__":
    PKG = 'jsk_pcl_ros_utils'
    ID = 'test_point_indices_to_cluster_point_indices'
    rospy.init_node(ID)
    rostest.rosrun(PKG, ID, TestPointIndicesToClusterPointIndices)
