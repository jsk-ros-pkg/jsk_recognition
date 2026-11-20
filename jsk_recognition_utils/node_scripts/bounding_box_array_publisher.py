#!/usr/bin/env python

import os
import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray

# ROS 1: rospy, as before; ROS 2: rclpy through jsk_ros1_ros2_compat
ROS_VERSION = int(os.environ.get('ROS_VERSION', '1'))
if ROS_VERSION == 1:
    import rospy
    from tf.transformations import quaternion_from_euler
else:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros
    from tf_transformations import quaternion_from_euler
    import yaml


class BoundingBoxArrayPublisher(object):

    def __init__(self, node=None):
        # node: the node of ROS 2 (None on ROS 1)
        self.node = node
        self.seq = 0

        if ROS_VERSION != 1:
            self._init_ros2()
            return

        self.frame_id = rospy.get_param('~frame_id')
        if (rospy.has_param('~positions') or
                rospy.has_param('~rotations') or
                rospy.has_param('~dimensions')):
            # Deprecated bounding box pose/dimension specification
            rospy.logwarn("DEPRECATION WARNING: Rosparam '~positions', "
                          "'~rotations' and '~dimensions' are being "
                          "deprecated. Please use '~boxes' instead.")
            positions = rospy.get_param('~positions')
            rotations = rospy.get_param('~rotations')
            dimensions = rospy.get_param('~dimensions')
            if len(rotations) != len(positions):
                rospy.logfatal('Number of ~rotations is expected as {}, but {}'
                            .format(len(positions), len(rotations)))
                sys.exit(1)
            if len(dimensions) != len(positions):
                rospy.logfatal('Number of ~dimensions is expected as {}, but {}'
                            .format(len(positions), len(dimensions)))
                sys.exit(1)
            self.boxes = []
            for pos, rot, dim in zip(positions, rotations, dimensions):
                self.boxes.append({
                    'position': pos,
                    'rotation': rot,
                    'dimension': dim,
                })
        else:
            self.boxes = rospy.get_param('~boxes')
        self.pub = rospy.Publisher('~output', BoundingBoxArray, queue_size=1)

        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def _init_ros2(self):
        # the deprecated ~positions, ~rotations and ~dimensions (lists of
        # lists, which a parameter of ROS 2 cannot be) are of ROS 1 only
        self.frame_id = ros.get_param(self.node, '~frame_id')
        self.boxes = ros.get_param(self.node, '~boxes')
        # a YAML string, as a parameter of ROS 2 cannot be a list of dicts
        if isinstance(self.boxes, str):
            self.boxes = yaml.safe_load(self.boxes)
        self.pub = ros.create_publisher(
            self.node, '~output', BoundingBoxArray, queue_size=1)

        rate = ros.get_param(self.node, '~rate', 1)
        self.timer = ros.create_timer(self.node, 1. / rate, self.publish)

    def publish(self, event):
        if ROS_VERSION == 1:
            stamp = event.current_real
        else:
            stamp = ros.stamp_now(self.node)
        bbox_array_msg = BoundingBoxArray()
        if ROS_VERSION == 1:
            bbox_array_msg.header.seq = self.seq
        bbox_array_msg.header.frame_id = self.frame_id
        bbox_array_msg.header.stamp = stamp
        for box in self.boxes:
            pos = box['position']
            rot = box.get('rotation', [0, 0, 0])
            qua = quaternion_from_euler(*rot)
            dim = box['dimension']
            label = box.get('label', 0)

            bbox_msg = BoundingBox()
            if ROS_VERSION == 1:
                bbox_msg.header.seq = self.seq
            bbox_msg.header.frame_id = self.frame_id
            bbox_msg.header.stamp = stamp
            bbox_msg.pose.position = Point(
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            bbox_msg.pose.orientation = Quaternion(
                x=float(qua[0]), y=float(qua[1]), z=float(qua[2]),
                w=float(qua[3]))
            bbox_msg.dimensions = Vector3(
                x=float(dim[0]), y=float(dim[1]), z=float(dim[2]))
            bbox_msg.label = label

            bbox_array_msg.boxes.append(bbox_msg)

        self.pub.publish(bbox_array_msg)


if __name__ == '__main__':
    if ROS_VERSION == 1:
        rospy.init_node('bounding_box_array_publisher')
        BoundingBoxArrayPublisher()
        rospy.spin()
    else:
        node = ros.init_node('bounding_box_array_publisher')
        BoundingBoxArrayPublisher(node)
        ros.spin(node)
