#!/usr/bin/env python

import os

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport

# ROS 1: rospy, as before; ROS 2: rclpy
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
else:
    import rclpy
    from rclpy.executors import ExternalShutdownException


class RectArrayToPolygonArray(ConnectionBasedTransport):

    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(RectArrayToPolygonArray, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(RectArrayToPolygonArray, self).__init__(
                'rect_array_to_polygon_array', **kwargs)
        self.pub = self.advertise('~output', PolygonArray, queue_size=1)

    def subscribe(self):
        if ROS_VERSION == 1:
            self.sub = rospy.Subscriber('~input', RectArray, self._convert)
        else:
            self.sub = self.create_subscription(
                RectArray, '~/input', self._convert, 1)

    def unsubscribe(self):
        if ROS_VERSION == 1:
            self.sub.unregister()
        else:
            self.destroy_subscription(self.sub)

    def _convert(self, msg):
        polys_msg = PolygonArray()
        polys_msg.header = msg.header
        for rect in msg.rects:
            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            x0, y0 = float(rect.x), float(rect.y)
            x1, y1 = float(rect.x + rect.width), float(rect.y + rect.height)
            pt0 = Point32(x=x0, y=y0)
            pt1 = Point32(x=x0, y=y1)
            pt2 = Point32(x=x1, y=y1)
            pt3 = Point32(x=x1, y=y0)
            poly_msg.polygon.points.append(pt0)
            poly_msg.polygon.points.append(pt1)
            poly_msg.polygon.points.append(pt2)
            poly_msg.polygon.points.append(pt3)
            polys_msg.polygons.append(poly_msg)
        self.pub.publish(polys_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('rect_array_to_polygon_array')
        RectArrayToPolygonArray()
        rospy.spin()
    else:
        rclpy.init()
        try:
            rclpy.spin(RectArrayToPolygonArray())
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
