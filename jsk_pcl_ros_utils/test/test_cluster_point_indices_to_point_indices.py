#!/usr/bin/env python

import unittest

import numpy as np

from jsk_recognition_msgs.msg import ClusterPointIndices
from pcl_msgs.msg import PointIndices
import dynamic_reconfigure.client
import rospy
import rostest


class TestClusterPointIndicesToPointIndices(unittest.TestCase):

    def setUp(self):
        self.msg = None
        self.cluster_indices = ((0, 10, 20, 30), (5, 15, 25, 35))
        self.dynparam = dynamic_reconfigure.client.Client(
            'cluster_point_indices_to_point_indices')
        self.pub = rospy.Publisher('~output', ClusterPointIndices,
                                   queue_size=1)
        self.sub = rospy.Subscriber('~input', PointIndices, self.cb)

    def cb(self, msg):
        self.msg = msg

    def test_conversion(self):
        self.dynparam.update_configuration({'index': 1})

        cluster_indices_msg = ClusterPointIndices()
        while self.msg is None:
            cluster_indices_msg.header.stamp = rospy.Time.now()
            for indices in self.cluster_indices:
                cluster_indices_msg.cluster_indices.append(
                    PointIndices(
                        header=cluster_indices_msg.header,
                        indices=indices,
                    )
                )
            self.pub.publish(cluster_indices_msg)
            rospy.sleep(0.1)

        self.assertTupleEqual(self.msg.indices, self.cluster_indices[1])


if __name__ == '__main__':
    PKG = 'jsk_pcl_ros_utils'
    ID = 'test_cluster_point_indices_to_point_indices'
    rospy.init_node(ID)
    rostest.rosrun(PKG, ID, TestClusterPointIndicesToPointIndices)
