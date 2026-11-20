#!/usr/bin/env python

import os

from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_topic_tools import ConnectionBasedTransport
import message_filters

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


class AddBoundingBoxArray(ConnectionBasedTransport):
    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(AddBoundingBoxArray, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(AddBoundingBoxArray, self).__init__(
                'add_bounding_box_array', **kwargs)
        self._pub = self.advertise('~output', BoundingBoxArray, queue_size=1)

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
                sub = message_filters.Subscriber(tp, BoundingBoxArray)
            else:
                sub = message_filters.Subscriber(self, BoundingBoxArray, tp)
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

    def _decompose(self, *bboxes_msgs):
        out_msg = BoundingBoxArray()
        out_msg.header = bboxes_msgs[0].header
        for bboxes_msg in bboxes_msgs:
            out_msg.boxes.extend(bboxes_msg.boxes)
        self._pub.publish(out_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('add_bounding_box_array')
        app = AddBoundingBoxArray()
        rospy.spin()
    else:
        ros.rclpy.init()
        ros.spin(AddBoundingBoxArray())
