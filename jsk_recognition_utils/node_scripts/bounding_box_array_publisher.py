#!/usr/bin/env python

import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
import rospy
from tf.transformations import quaternion_from_euler


class BoundingBoxArrayPublisher(object):

    def __init__(self):
        self.seq = 0

        self.frame_id = rospy.get_param('~frame_id')
        self.positions = rospy.get_param('~positions')
        self.rotations = rospy.get_param('~rotations')
        self.dimensions = rospy.get_param('~dimensions')
        self.n_boxes = len(self.positions)
        if len(self.rotations) != self.n_boxes:
            rospy.logfatal('Number of ~rotations is expected as {}, but {}'
                           .format(self.n_boxes, len(self.rotations)))
            sys.exit(1)
        if len(self.dimensions) != self.n_boxes:
            rospy.logfatal('Number of ~dimensions is expected as {}, but {}'
                           .format(self.n_boxes, len(self.dimensions)))
            sys.exit(1)
        self.pub = rospy.Publisher('~output', BoundingBoxArray, queue_size=1)

        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def publish(self, event):
        bbox_array_msg = BoundingBoxArray()
        bbox_array_msg.header.seq = self.seq
        bbox_array_msg.header.frame_id = self.frame_id
        bbox_array_msg.header.stamp = event.current_real
        for i_box in xrange(self.n_boxes):
            pos = self.positions[i_box]
            rot = self.rotations[i_box]
            qua = quaternion_from_euler(*rot)
            dim = self.dimensions[i_box]

            bbox_msg = BoundingBox()
            bbox_msg.header.seq = self.seq
            bbox_msg.header.frame_id = self.frame_id
            bbox_msg.header.stamp = event.current_real
            bbox_msg.pose.position = Point(*pos)
            bbox_msg.pose.orientation = Quaternion(*qua)
            bbox_msg.dimensions = Vector3(*dim)

            bbox_array_msg.boxes.append(bbox_msg)

        self.pub.publish(bbox_array_msg)


if __name__ == '__main__':
    rospy.init_node('bounding_box_array_publisher')
    BoundingBoxArrayPublisher()
    rospy.spin()
