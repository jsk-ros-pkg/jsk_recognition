#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import tf2_ros
import tf2_geometry_msgs

from jsk_recognition_utils.panorama_utils import transform_rect_in_panorama_to_bounding_box

import sys

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import PanoramaInfo
from jsk_recognition_msgs.msg import BoundingBoxArray
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

        try:
            frame_fixed_to_panorama = tf2_geometry_msgs.transform_to_kdl(
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

        msg_bbox_array = transform_rect_in_panorama_to_bounding_box(
            [track.rect for track in msg.tracks],
            [track.class_name for track in msg.tracks],
            [track.track_id for track in msg.tracks],
            [0 for track in msg.tracks],
            frame_fixed_to_panorama,
            self._frame_fixed,
            time_current,
            self._dimensions_labels,
            self._image_width,
            self._image_height,
            self._theta_min,
            self._theta_max,
            self._phi_min,
            self._phi_max
        )

        self._pub_bbox_array.publish(msg_bbox_array)


def main():
    rospy.init_node('track_array_in_panorama_to_bounding_box_array')
    node = TrackArrayInPanoramaToBoundingBoxArray()
    rospy.spin()


if __name__ == '__main__':
    main()
