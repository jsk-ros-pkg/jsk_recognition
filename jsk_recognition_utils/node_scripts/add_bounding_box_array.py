#!/usr/bin/env python

import os

from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_topic_tools import ConnectionBasedTransport
import message_filters

# ROS 1: rospy, as before; ROS 2: rclpy
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter


class AddBoundingBoxArray(ConnectionBasedTransport):
    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(AddBoundingBoxArray, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(AddBoundingBoxArray, self).__init__(
                'add_bounding_box_array', **kwargs)
            self.declare_parameters('', [
                ('topics', Parameter.Type.STRING_ARRAY),
                ('approximate_sync', False),
                ('queue_size', 100),
                ('slop', 0.1),
            ])
        self._pub = self.advertise('~output', BoundingBoxArray, queue_size=1)

    def subscribe(self):
        if ROS_VERSION == 1:
            topics = rospy.get_param('~topics')
            use_async = rospy.get_param('~approximate_sync', False)
            queue_size = rospy.get_param('~queue_size', 100)
            slop = rospy.get_param('~slop', 0.1)
        else:
            topics = self.get_parameter('topics').value
            use_async = self.get_parameter('approximate_sync').value
            queue_size = self.get_parameter('queue_size').value
            slop = self.get_parameter('slop').value
        self.subs = []
        for tp in topics:
            sub = message_filters.Subscriber(tp, BoundingBoxArray)
            self.subs.append(sub)
        if use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._decompose)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

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
        rclpy.init()
        try:
            rclpy.spin(AddBoundingBoxArray())
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
