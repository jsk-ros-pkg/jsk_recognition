#!/usr/bin/env python

import os

import numpy as np

import message_filters

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
from pcl_msgs.msg import PointIndices
from sensor_msgs.msg import CameraInfo

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


class RectArrayToClusterPointIndices(ConnectionBasedTransport):

    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(RectArrayToClusterPointIndices, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(RectArrayToClusterPointIndices, self).__init__(
                'rect_array_to_cluster_point_indices', **kwargs)
        self.pub = self.advertise('~output', ClusterPointIndices, queue_size=1)

    def subscribe(self):
        if ROS_VERSION == 1:
            use_info = rospy.get_param('~use_info', False)
        else:
            use_info = ros.get_param(self, '~use_info', False)
        if use_info:
            if ROS_VERSION == 1:
                sub_rects = message_filters.Subscriber('~input', RectArray)
                sub_info = message_filters.Subscriber(
                    '~input/info', CameraInfo)
                queue_size = rospy.get_param('~queue_size', 10)
                approximate_sync = rospy.get_param('~approximate_sync', False)
                slop = rospy.get_param('~slop', 0.1)
            else:
                sub_rects = message_filters.Subscriber(
                    self, RectArray, '~/input')
                sub_info = message_filters.Subscriber(
                    self, CameraInfo, '~/input/info')
                queue_size = ros.get_param(self, '~queue_size', 10)
                approximate_sync = ros.get_param(
                    self, '~approximate_sync', False)
                slop = ros.get_param(self, '~slop', 0.1)
            self.subs = [sub_rects, sub_info]
            if approximate_sync:
                sync = message_filters.ApproximateTimeSynchronizer(
                    self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    self.subs, queue_size=queue_size)
            sync.registerCallback(self._convert_sync)
        else:
            if ROS_VERSION == 1:
                self.img_height = rospy.get_param('~img_height')
                self.img_width = rospy.get_param('~img_width')
                sub_rects = rospy.Subscriber(
                    '~input', RectArray, self._convert)
            else:
                self.img_height = ros.get_param(self, '~img_height')
                self.img_width = ros.get_param(self, '~img_width')
                sub_rects = ros.create_subscription(
                    self, '~input', RectArray, self._convert)
            self.subs = [sub_rects]

    def unsubscribe(self):
        for sub in self.subs:
            if ROS_VERSION == 1:
                sub.unregister()
            elif isinstance(sub, message_filters.Subscriber):
                self.destroy_subscription(sub.sub)
            else:
                self.destroy_subscription(sub)

    def _convert_sync(self, rects_msg, info_msg):
        H = info_msg.height
        W = info_msg.width
        self._convert(rects_msg, H, W)

    def _convert(self, rects_msg, img_height=None, img_width=None):
        H = self.img_height if img_height is None else img_height
        W = self.img_width if img_width is None else img_width
        cpi_msg = ClusterPointIndices(header=rects_msg.header)
        for rect in rects_msg.rects:
            indices_msg = PointIndices(header=rects_msg.header)
            ymin = max(0, int(np.floor(rect.y)))
            xmin = max(0, int(np.floor(rect.x)))
            ymax = min(H, int(np.ceil(rect.y + rect.height)))
            xmax = min(W, int(np.ceil(rect.x + rect.width)))
            indices = [list(range(W*y+xmin, W*y+xmax)) for y in range(ymin, ymax)]
            indices_msg.indices = np.array(
                indices, dtype=np.int32).flatten().tolist()
            cpi_msg.cluster_indices.append(indices_msg)
        self.pub.publish(cpi_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('rect_array_to_cluster_point_indices')
        RectArrayToClusterPointIndices()
        rospy.spin()
    else:
        ros.rclpy.init()
        ros.spin(RectArrayToClusterPointIndices())
