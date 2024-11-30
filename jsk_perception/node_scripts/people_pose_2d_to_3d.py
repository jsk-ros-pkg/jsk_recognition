#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function

import numpy as np

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class PeoplePose2Dto3D(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.sub_info = None
        self.pose_pub = self.advertise(
            '~output/pose', PeoplePoseArray, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_pose = message_filters.Subscriber(
            '~input/pose', PeoplePoseArray, queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            '~input/depth', Image, queue_size=1, buff_size=2**24)
        self.subs = [sub_pose, sub_depth]

        # NOTE: Camera info is not synchronized by default.
        # See https://github.com/jsk-ros-pkg/jsk_recognition/issues/2165
        sync_cam_info = rospy.get_param("~sync_camera_info", False)
        if sync_cam_info:
            sub_info = message_filters.Subscriber(
                '~input/info', CameraInfo, queue_size=1, buff_size=2**24)
            self.subs.append(sub_info)
        else:
            self.sub_info = rospy.Subscriber(
                '~input/info', CameraInfo, self._cb_cam_info)

        if rospy.get_param('~approximate_sync', True):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        if sync_cam_info:
            sync.registerCallback(self._cb_with_depth_info)
        else:
            self.camera_info_msg = None
            sync.registerCallback(self._cb_with_depth)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()
        if self.sub_info is not None:
            self.sub_info.unregister()
            self.sub_info = None

    def _cb_cam_info(self, msg):
        self.camera_info_msg = msg
        self.sub_info.unregister()
        self.sub_info = None
        rospy.loginfo("Received camera info")

    def _cb_with_depth(self, pose_2d_array_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(
            pose_2d_array_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(
            self, pose_2d_array_msg, depth_msg, camera_info_msg
    ):
        br = cv_bridge.CvBridge()
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        pose_3d_array_msg = PeoplePoseArray()
        pose_3d_array_msg.header = pose_2d_array_msg.header

        # calculate xyz-position
        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]

        for pose_2d_msg in pose_2d_array_msg.poses:
            limb_names = pose_2d_msg.limb_names
            scores = pose_2d_msg.scores
            poses = pose_2d_msg.poses
            pose_3d_msg = PeoplePose()
            for limb_name, score, pose in zip(limb_names, scores, poses):
                position = pose.position
                if score < 0:
                    continue
                if 0 <= position.y < depth_img.shape[0] and\
                   0 <= position.x < depth_img.shape[1]:
                    z = float(depth_img[int(position.y)][int(position.x)])
                else:
                    continue
                if np.isnan(z):
                    continue
                x = (position.x - cx) * z / fx
                y = (position.y - cy) * z / fy
                pose_3d_msg.limb_names.append(limb_name)
                pose_3d_msg.scores.append(score)
                pose_3d_msg.poses.append(
                    Pose(position=Point(x=x, y=y, z=z),
                         orientation=Quaternion(w=1)))
            pose_3d_array_msg.poses.append(pose_3d_msg)
        self.pose_pub.publish(pose_3d_array_msg)


if __name__ == '__main__':
    rospy.init_node('people_pose_2d_to_3d')
    PeoplePose2Dto3D()
    rospy.spin()
