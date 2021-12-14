#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import PyKDL

import tf2_ros
import tf2_geometry_msgs

from jsk_perception.lib import transformPanoramaPoint, calcSphericalPoint

import math
import sys

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import PanoramaInfo
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import TrackArray


class TrackArrayInPanoramaToBoundingBoxArray(object):

    def __init__(self):

        self._frame_fixed = rospy.get_param('~frame_fixed', 'fixed_frame')
        self._dimensions_labels = rospy.get_param('~dimensions_labels', {})
        self._duration_timeout = rospy.get_param('~duration_timeout', 0.05)

        try:
            msg_panorama_image = rospy.wait_for_message(
                '~panorama_image', Image, 10)
            msg_panorama_info = rospy.wait_for_message(
                '~panorama_info', PanoramaInfo, 10)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            sys.exit(1)

        # Parameters
        self._frame_panorama = msg_panorama_info.header.frame_id
        self._theta_min = msg_panorama_info.theta_min
        self._theta_max = msg_panorama_info.theta_max
        self._phi_min = msg_panorama_info.phi_min
        self._phi_max = msg_panorama_info.phi_max
        self._image_height = msg_panorama_image.height
        self._image_width = msg_panorama_image.width

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Publisher
        self._pub_bbox_array = rospy.Publisher(
            '~bbox_array', BoundingBoxArray, queue_size=1)

        # Subscriber
        self._sub = rospy.Subscriber(
            '~input_tracks', TrackArray, self.callback)

    def callback(self, msg):

        time_current = msg.header.stamp

        msg_bbox_array = BoundingBoxArray()
        msg_bbox_array.header.frame_id = self._frame_fixed
        msg_bbox_array.header.stamp = msg.header.stamp

        try:
            pykdl_transform_fixed_to_panorama = \
                tf2_geometry_msgs.transform_to_kdl(
                    self._tf_buffer.lookup_transform(
                        self._frame_fixed,
                        self._frame_panorama,
                        time_current,
                        timeout=rospy.Duration(self._duration_timeout)
                    )
                )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        for track in msg.tracks:

            if track.class_name not in self._dimensions_labels:
                rospy.logwarn(
                    'height for label "{}" (id:{}) is not specified'.format(
                        track.class_name, track.class_id))
                continue

            (theta_a, phi_a) = transformPanoramaPoint(
                track.rect.x,
                track.rect.y,
                self._image_height,
                self._image_width,
                self._theta_min,
                self._theta_max,
                self._phi_min,
                self._phi_max
            )
            (theta_b, phi_b) = transformPanoramaPoint(
                track.rect.x + track.rect.width,
                track.rect.y + track.rect.height,
                self._image_height,
                self._image_width,
                self._theta_min,
                self._theta_max,
                self._phi_min,
                self._phi_max
            )
            theta = (theta_a + theta_b) / 2.0
            phi = (phi_a + phi_b) / 2.0
            distance = (self._dimensions_labels[track.class_name][2] / 2.0) \
                / math.tan((theta_b - theta_a) / 2.0)

            (x, y, z) = calcSphericalPoint(theta, phi, distance)
            position_fixedbased = pykdl_transform_fixed_to_panorama * \
                PyKDL.Vector(x, y, z)

            msg_bbox = BoundingBox()
            msg_bbox.header = msg_bbox_array.header
            msg_bbox.pose.position.x = position_fixedbased[0]
            msg_bbox.pose.position.y = position_fixedbased[1]
            msg_bbox.pose.position.z = position_fixedbased[2]
            msg_bbox.pose.orientation.w = 1.0
            msg_bbox.dimensions.x = \
                self._dimensions_labels[track.class_name][0]
            msg_bbox.dimensions.y = \
                self._dimensions_labels[track.class_name][1]
            msg_bbox.dimensions.z = \
                self._dimensions_labels[track.class_name][2]
            msg_bbox.label = track.track_id

            msg_bbox_array.boxes.append(msg_bbox)

        self._pub_bbox_array.publish(msg_bbox_array)


def main():
    rospy.init_node('track_array_in_panorama_to_bounding_box_array')
    node = TrackArrayInPanoramaToBoundingBoxArray()
    rospy.spin()


if __name__ == '__main__':
    main()
