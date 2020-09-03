#!/usr/bin/env python

import numpy as np

import message_filters
import rospy

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
from pcl_msgs.msg import PointIndices
from sensor_msgs.msg import CameraInfo


class RectArrayToClusterPointIndices(ConnectionBasedTransport):

    def __init__(self):
        super(RectArrayToClusterPointIndices, self).__init__()
        self.pub = self.advertise('~output', ClusterPointIndices, queue_size=1)

    def subscribe(self):
        if rospy.get_param('~use_info', False):
            sub_rects = message_filters.Subscriber('~input', RectArray)
            sub_info = message_filters.Subscriber('~input/info', CameraInfo)
            self.subs = [sub_rects, sub_info]
            queue_size = rospy.get_param('~queue_size', 10)
            if rospy.get_param('~approximate_sync', False):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    self.subs, queue_size=queue_size)
            sync.registerCallback(self._convert_sync)
        else:
            self.img_height = rospy.get_param('~img_height')
            self.img_width = rospy.get_param('~img_width')
            sub_rects = rospy.Subscriber('~input', RectArray, self._convert)
            self.subs = [sub_rects]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

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
            indices = [range(W*y+xmin, W*y+xmax) for y in range(ymin, ymax)]
            indices_msg.indices = np.array(indices, dtype=np.int32).flatten()
            cpi_msg.cluster_indices.append(indices_msg)
        self.pub.publish(cpi_msg)


if __name__ == '__main__':
    rospy.init_node('rect_array_to_cluster_point_indices')
    RectArrayToClusterPointIndices()
    rospy.spin()
