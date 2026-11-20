#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_topic_tools import ConnectionBasedTransport
import message_filters

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


class AddClusterIndices(ConnectionBasedTransport):
    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(AddClusterIndices, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(AddClusterIndices, self).__init__(
                'add_cluster_indices', **kwargs)
        self._pub = self.advertise('~output', ClusterPointIndices,
                                   queue_size=1)

    def subscribe(self):
        if ROS_VERSION == 1:
            topics = rospy.get_param('~topics')
            use_async = rospy.get_param('~approximate_sync', False)
            queue_size = rospy.get_param('~queue_size', 100)
            slop = rospy.get_param('~slop', 0.1)
        else:
            topics = ros.get_param(self, '~topics')
            use_async = ros.get_param(self, '~approximate_sync', False)
            queue_size = ros.get_param(self, '~queue_size', 100)
            slop = ros.get_param(self, '~slop', 0.1)
        self.subs = []
        for tp in topics:
            if ROS_VERSION == 1:
                sub = message_filters.Subscriber(tp, ClusterPointIndices)
            else:
                sub = message_filters.Subscriber(self, ClusterPointIndices, tp)
            self.subs.append(sub)
        if use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._decompose)

    def unsubscribe(self):
        for sub in self.subs:
            if ROS_VERSION == 1:
                sub.unregister()
            else:
                self.destroy_subscription(sub.sub)

    def _decompose(self, *cpi_msgs):
        out_msg = ClusterPointIndices()
        out_msg.header = cpi_msgs[0].header
        for cpi_msg in cpi_msgs:
            out_msg.cluster_indices.extend(cpi_msg.cluster_indices)
        self._pub.publish(out_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('add_cluster_indices')
        app = AddClusterIndices()
        rospy.spin()
    else:
        ros.rclpy.init()
        ros.spin(AddClusterIndices())
