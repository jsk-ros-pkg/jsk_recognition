#!/usr/bin/env python

from __future__ import division

from geometry_msgs.msg import PoseStamped
from pcl_msgs.msg import PointIndices
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class SampleTopicEnsyncForCaptureStereoSynchronizer(object):

    def __init__(self):
        self.pub_pose = rospy.Publisher(
            '~output/pose', PoseStamped, queue_size=1)
        self.pub_mask = rospy.Publisher(
            '~output/mask', Image, queue_size=1)
        self.pub_mask_indices = rospy.Publisher(
            '~output/mask_indices', PointIndices, queue_size=1)
        self.pub_left_image = rospy.Publisher(
            '~output/left_image', Image, queue_size=1)
        self.pub_left_camera_info = rospy.Publisher(
            '~output/left_camera_info', CameraInfo, queue_size=1)
        self.pub_right_camera_info = rospy.Publisher(
            '~output/right_camera_info', CameraInfo, queue_size=1)
        self.pub_disparity = rospy.Publisher(
            '~output/disparity', DisparityImage, queue_size=1)

        self.latest_pose = None
        self.latest_mask = None
        self.latest_mask_indices = None
        self.latest_left_image = None
        self.latest_left_camera_info = None
        self.latest_right_camera_info = None
        self.latest_disparity = None

        rospy.Subscriber('~input/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('~input/mask', Image, self._mask_cb)
        rospy.Subscriber('~input/mask_indices', PointIndices, self._indices_cb)
        rospy.Subscriber('~input/left_image', Image, self._left_image_cb)
        rospy.Subscriber(
            '~input/left_camera_info', CameraInfo, self._left_camera_info_cb)
        rospy.Subscriber(
            '~input/right_camera_info', CameraInfo, self._right_camera_info_cb)
        rospy.Subscriber(
            '~input/disparity', DisparityImage, self._disparity_cb)

        r = rospy.Rate(rospy.get_param('~rate', 10))
        while not rospy.is_shutdown():
            self._publish()
            try:
                r.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                pass

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _mask_cb(self, msg):
        self.latest_mask = msg

    def _indices_cb(self, msg):
        self.latest_mask_indices = msg

    def _left_image_cb(self, msg):
        self.latest_left_image = msg

    def _left_camera_info_cb(self, msg):
        self.latest_left_camera_info = msg

    def _right_camera_info_cb(self, msg):
        self.latest_right_camera_info = msg

    def _disparity_cb(self, msg):
        self.latest_disparity = msg

    def _publish(self):
        if self.latest_pose is None or \
           self.latest_mask is None or \
           self.latest_mask_indices is None or \
           self.latest_left_image is None or \
           self.latest_left_camera_info is None or \
           self.latest_right_camera_info is None or \
           self.latest_disparity is None:
            return

        now = rospy.Time.now()
        self.latest_pose.header.stamp = now
        self.latest_mask.header.stamp = now
        self.latest_mask_indices.header.stamp = now
        self.latest_left_image.header.stamp = now
        self.latest_left_camera_info.header.stamp = now
        self.latest_right_camera_info.header.stamp = now
        self.latest_disparity.header.stamp = now

        self.pub_pose.publish(self.latest_pose)
        self.pub_mask.publish(self.latest_mask)
        self.pub_mask_indices.publish(self.latest_mask_indices)
        self.pub_left_image.publish(self.latest_left_image)
        self.pub_left_camera_info.publish(self.latest_left_camera_info)
        self.pub_right_camera_info.publish(self.latest_right_camera_info)
        self.pub_disparity.publish(self.latest_disparity)


if __name__ == '__main__':
    rospy.init_node('sample_topic_ensync_for_capture_stereo_synchronizer')
    app = SampleTopicEnsyncForCaptureStereoSynchronizer()
    rospy.spin()
