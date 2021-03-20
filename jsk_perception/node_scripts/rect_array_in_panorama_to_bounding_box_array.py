#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy
import message_filters
import PyKDL

import numpy as np

import tf2_ros
import tf2_geometry_msgs

import math
import sys
import threading

from sound_play.libsoundplay import SoundClient

from sensor_msgs.msg import Image, PanoramaInfo
from jsk_recognition_msgs.msg import RectArray, ClassificationResult
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Twist, PointStamped, PoseStamped

def transformPanoramaPoint(x,
                           y,
                           image_height,
                           image_width,
                           theta_min,
                           theta_max,
                           phi_min,
                           phi_max):
    phi = phi_max + 1.0 * (phi_min - phi_max) * x / image_width
    theta = theta_min + 1.0 * (theta_max - theta_min) * y / image_height
    return (theta, phi)


def calcSphericalPoint( theta, phi, r ):
    return ( r * math.sin(theta) * math.cos(phi),
             r * math.sin(theta) * math.sin(phi),
             r * math.cos(theta) )


class RectArrayInPanoramaToBoundingBoxArray(object):

    def __init__(self):

        self._frame_fixed = rospy.get_param( '~frame_fixed', 'fixed_frame' )
        self._dimensions_labels = rospy.get_param( '~dimensions_labels', {} )
        self._duration_timeout = rospy.get_param( '~duration_timeout', 0.05 )

        try:
            msg_panorama_image = rospy.wait_for_message(
                '~panorama_image', Image, 10)
            msg_panorama_info = rospy.wait_for_message(
                '~panorama_info', PanoramaInfo, 10)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logerr('{}'.format(e))
            sys.exit(1)

        self._frame_panorama = msg_panorama_info.header.frame_id
        self._theta_min = msg_panorama_info.theta_min
        self._theta_max = msg_panorama_info.theta_max
        self._phi_min = msg_panorama_info.phi_min
        self._phi_max = msg_panorama_info.phi_max
        self._image_height = msg_panorama_image.height
        self._image_width = msg_panorama_image.width
 
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Publisher
        self._pub_bbox_array = rospy.Publisher( '~bbox_array', BoundingBoxArray, queue_size=1 )

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message( '~input_class', ClassificationResult, 3)
                rospy.wait_for_message( '~input_rects', RectArray, 3)
                break
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn('subscribing topic seems not to be pulished. waiting... Error: {}'.format(e))
            rate.sleep()

        # Subscriber
        mf_sub_class = message_filters.Subscriber( '~input_class', ClassificationResult )
        mf_sub_rects = message_filters.Subscriber( '~input_rects', RectArray )

        ts = message_filters.TimeSynchronizer([mf_sub_class,mf_sub_rects],10)
        ts.registerCallback(self._cb_object)

        rospy.loginfo('Node is successfully initialized')

    def _cb_object(self,
            msg_class,
            msg_rects):

        rospy.loginfo('callback called')

        time_current = msg_rects.header.stamp

        msg_bbox_array = BoundingBoxArray()
        msg_bbox_array.header.frame_id = self._frame_fixed
        msg_bbox_array.header.stamp = msg_rects.header.stamp

        try:
            pykdl_transform_fixed_to_panorama = tf2_geometry_msgs.transform_to_kdl(
                        self._tf_buffer.lookup_transform(
                            self._frame_fixed,
                            self._frame_panorama,
                            time_current,
                            timeout=rospy.Duration(self._duration_timeout)
                            )
                    )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('{}'.format(e))
            return

        for label_id, \
            label_name, \
            rect in zip(msg_class.labels,\
                        msg_class.label_names,\
                        msg_rects.rects):

            if label_name not in self._dimensions_labels:
                rospy.logwarn('height for label "{}" (id:{}) is not specified'.format(label_name,label_id))
                continue

            (theta_a, phi_a) = transformPanoramaPoint(
                                    rect.x,
                                    rect.y,
                                    self._image_height,
                                    self._image_width,
                                    self._theta_min,
                                    self._theta_max,
                                    self._phi_min,
                                    self._phi_max
                                    )
            (theta_b, phi_b) = transformPanoramaPoint(
                                    rect.x + rect.width,
                                    rect.y + rect.height,
                                    self._image_height,
                                    self._image_width,
                                    self._theta_min,
                                    self._theta_max,
                                    self._phi_min,
                                    self._phi_max
                                    )
            theta = (theta_a + theta_b) / 2.0
            phi = (phi_a + phi_b ) / 2.0
            width_in_rad = phi_b - phi_a
            height_in_rad = theta_b - theta_a
            distance = (self._dimensions_labels[label_name][2] / 2.0 ) / math.tan( (theta_b - theta_a) / 2.0 )

            (x,y,z) = calcSphericalPoint( theta, phi, distance )
            position_fixedbased = pykdl_transform_fixed_to_panorama * PyKDL.Vector(x,y,z)

            msg_bbox = BoundingBox()
            msg_bbox.header = msg_bbox_array.header
            msg_bbox.pose.position.x = position_fixedbased[0]
            msg_bbox.pose.position.y = position_fixedbased[1]
            msg_bbox.pose.position.z = position_fixedbased[2]
            msg_bbox.pose.orientation.w = 1.0
            msg_bbox.dimensions.x = self._dimensions_labels[label_name][0]
            msg_bbox.dimensions.y = self._dimensions_labels[label_name][1]
            msg_bbox.dimensions.z = self._dimensions_labels[label_name][2]
            msg_bbox.label = label_id

            msg_bbox_array.boxes.append(msg_bbox)

        self._pub_bbox_array.publish(msg_bbox_array)


def main():

    rospy.init_node('rect_array_in_panorama_to_bounding_box_array')
    node = RectArrayInPanoramaToBoundingBoxArray()
    rospy.spin()


if __name__=='__main__':
    main()
