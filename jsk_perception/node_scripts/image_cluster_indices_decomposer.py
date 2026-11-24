#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv_bridge
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image


class ImageClusterIndicesDecomposer(ConnectionBasedTransport):
    def __init__(self):
        super(ImageClusterIndicesDecomposer, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self._sub = message_filters.Subscriber('~input', Image)
        self._sub_cpi = message_filters.Subscriber('~input/cluster_indices',
                                                   ClusterPointIndices)
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 100)
        subs = [self._sub, self._sub_cpi]
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(subs, queue_size)
        sync.registerCallback(self._decompose)

    def unsubscribe(self):
        self._sub.unregister()
        self._sub_cpi.unregister()

    def _decompose(self, imgmsg, cpi_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')

        from jsk_recognition_utils import colorize_cluster_indices
        cluster_indices = [list(i_msg.indices)
                           for i_msg in cpi_msg.cluster_indices]
        out = colorize_cluster_indices(img, cluster_indices)

        out_msg = bridge.cv2_to_imgmsg(out, encoding='rgb8')
        out_msg.header = imgmsg.header
        self._pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('image_cluster_indices_decomposer')
    app = ImageClusterIndicesDecomposer()
    rospy.spin()
