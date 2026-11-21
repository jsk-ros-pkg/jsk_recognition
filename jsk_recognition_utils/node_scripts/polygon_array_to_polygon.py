#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from jsk_topic_tools import ConnectionBasedTransport

# ROS 1: rospy, as before; ROS 2: rclpy
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import dynamic_reconfigure.server
    from jsk_recognition_utils.cfg import PolygonArrayToPolygonConfig
    from jsk_topic_tools.log_utils import logerr_throttle
    import rospy
else:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import SetParametersResult
    from types import SimpleNamespace


class PolygonArrayToPolygon(ConnectionBasedTransport):

    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(self.__class__, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(self.__class__, self).__init__(
                'polygon_array_to_polygon', **kwargs)
        self.pub = self.advertise('~output', PolygonStamped, queue_size=1)
        if ROS_VERSION == 1:
            dynamic_reconfigure.server.Server(PolygonArrayToPolygonConfig,
                                              self._config_callback)
        else:
            # the parameter of cfg/PolygonArrayToPolygon.cfg, with its default
            self.declare_parameter('index', -1)
            self._config_callback(
                SimpleNamespace(index=self.get_parameter('index').value), 0)
            self.add_on_set_parameters_callback(self._set_parameters_cb)

    def subscribe(self):
        if ROS_VERSION == 1:
            self.sub = rospy.Subscriber('~input', PolygonArray, self._convert)
        else:
            self.sub = self.create_subscription(
                PolygonArray, '~/input', self._convert, 1)

    def unsubscribe(self):
        if ROS_VERSION == 1:
            self.sub.unregister()
        else:
            self.destroy_subscription(self.sub)

    def _config_callback(self, config, level):
        self.index = config.index
        return config

    def _set_parameters_cb(self, params):
        """ROS 2: `ros2 param set` of the parameter of cfg/"""
        for param in params:
            if param.name == 'index':
                self._config_callback(SimpleNamespace(index=param.value), 0)
        return SetParametersResult(successful=True)

    def _convert(self, msg):
        polygon_msg = PolygonStamped()
        polygon_msg.header = msg.header
        if self.index < 0:
            return
        elif self.index < len(msg.polygons):
            polygon_msg = msg.polygons[self.index]
            self.pub.publish(polygon_msg)
        else:
            text = ('Invalid index {} is specified '
                    'for polygon array whose size is {}'
                    .format(self.index, len(msg.polygons)))
            if ROS_VERSION == 1:
                logerr_throttle(10, text)
            else:
                self.get_logger().error(text, throttle_duration_sec=10)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('polygon_array_to_polygon')
        PolygonArrayToPolygon()
        rospy.spin()
    else:
        rclpy.init()
        try:
            rclpy.spin(PolygonArrayToPolygon())
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
