#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import EdgeArray, PolygonArray, PoseLabeledArray, ClusterPointIndices
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logerr_throttle
from jsk_topic_tools import jsk_loginfo
from jsk_topic_tools import warn_no_remap
import message_filters
import rospy
import numpy as np


class DummyElementPublisher(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()

        self.publish_edge = rospy.get_param('~publish_edge', False)
        self.publish_polygon = rospy.get_param('~publish_polygon', False)
        self.publish_pose = rospy.get_param('~publish_pose', False)

        if self.publish_edge:
            self.pub_edges = self.advertise('~output_edges', EdgeArray, queue_size=1)
            self.pub_edge_indices = self.advertise('~output_edge_indices', ClusterPointIndices, queue_size=1)
        if self.publish_polygon:
            self.pub_polys = self.advertise('~output_polygons', PolygonArray, queue_size=1)
            self.pub_poly_indices = self.advertise('~output_polygon_indices', ClusterPointIndices, queue_size=1)
        if self.publish_pose:
            self.pub_poses = self.advertise('~output_poses', PoseLabeledArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber("~input", PointCloud2, self.cb)

    def unsubscribe(self):
        self.sub.unregister()

    def cb(self, msg):
        if self.publish_edge:
            out_msg_edges = EdgeArray()
            out_msg_edges.header = msg.header
            self.pub_edges.publish(out_msg_edges)

            out_msg_edge_indices = ClusterPointIndices()
            out_msg_edge_indices.header = msg.header
            self.pub_edge_indices.publish(out_msg_edge_indices)

        if self.publish_polygon:
            out_msg_polys = PolygonArray()
            out_msg_polys.header = msg.header
            self.pub_polys.publish(out_msg_polys)

            out_msg_poly_indices = ClusterPointIndices()
            out_msg_poly_indices.header = msg.header
            self.pub_poly_indices.publish(out_msg_poly_indices)

        if self.publish_pose:
            out_msg_poses = PoseLabeledArray()
            out_msg_poses.header = msg.header
            self.pub_poses.publish(out_msg_poses)


if __name__ == '__main__':
    rospy.init_node('dummy_element_publisher')
    DummyElementPublisher()
    rospy.spin()
