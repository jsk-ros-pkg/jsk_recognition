#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from jsk_topic_tools import ConnectionBasedTransport

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import dynamic_reconfigure.server
    from jsk_recognition_utils.cfg import PoseArrayToPoseConfig
    from jsk_topic_tools.log_utils import logerr_throttle
    import rospy
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros


class PoseArrayToPose(ConnectionBasedTransport):

    def __init__(self, **kwargs):
        if ROS_VERSION == 1:
            super(self.__class__, self).__init__()
        else:
            # the transport is the node on ROS 2
            super(self.__class__, self).__init__(
                'pose_array_to_pose', **kwargs)
        self.pub = self.advertise('~output', PoseStamped, queue_size=1)
        if ROS_VERSION == 1:
            dynamic_reconfigure.server.Server(PoseArrayToPoseConfig,
                                              self._config_callback)
        else:
            # the parameters of cfg/PoseArrayToPose.cfg, its defaults
            self._srv = ros.create_reconfigure_server(
                self, None, self._config_callback, defaults={'index': -1})

    def subscribe(self):
        if ROS_VERSION == 1:
            self.sub = rospy.Subscriber('~input', PoseArray, self._convert)
        else:
            self.sub = ros.create_subscription(
                self, '~input', PoseArray, self._convert)

    def unsubscribe(self):
        if ROS_VERSION == 1:
            self.sub.unregister()
        else:
            self.destroy_subscription(self.sub)

    def _config_callback(self, config, level):
        self.index = config.index
        return config

    def _convert(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        if self.index < 0:
            return
        elif self.index < len(msg.poses):
            pose_msg.pose = msg.poses[self.index]
            self.pub.publish(pose_msg)
        else:
            text = ('Invalid index {} is specified '
                    'for pose array whose size is {}'
                    .format(self.index, len(msg.poses)))
            if ROS_VERSION == 1:
                logerr_throttle(10, text)
            else:
                ros.logerr_throttle(self, 10, text)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('pose_array_to_pose')
        PoseArrayToPose()
        rospy.spin()
    else:
        ros.rclpy.init()
        ros.spin(PoseArrayToPose())
