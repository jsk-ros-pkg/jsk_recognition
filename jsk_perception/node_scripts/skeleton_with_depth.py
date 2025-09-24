#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division
from __future__ import print_function

import cv_bridge
from image_geometry import PinholeCameraModel
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class SkeletonWithDepth(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.skeleton_pub = self.advertise(
            '~output/skeleton', HumanSkeletonArray, queue_size=1)
        self.bridge = cv_bridge.CvBridge()

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_skeleton = message_filters.Subscriber(
            '~input/skeleton',
            HumanSkeletonArray,
            queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            '~input/depth',
            Image,
            queue_size=1, buff_size=2**24)
        self.subs = [sub_skeleton, sub_depth]

        # NOTE: Camera info is not synchronized by default.
        # See https://github.com/jsk-ros-pkg/jsk_recognition/issues/2165
        sync_cam_info = rospy.get_param("~sync_camera_info", False)
        if sync_cam_info:
            sub_info = message_filters.Subscriber(
                '~input/info',
                CameraInfo, queue_size=1, buff_size=2**24)
            self.subs.append(sub_info)
        else:
            self.sub_info = rospy.Subscriber(
                '~input/info',
                CameraInfo, self._cb_cam_info)

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

    def _cb_with_depth(self, skeleton_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(skeleton_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(self, skeleton_msg, depth_msg, camera_info_msg):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_msg)
        br = cv_bridge.CvBridge()
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        H, W = depth_img.shape
        out_skeleton_array_msg = HumanSkeletonArray(header=skeleton_msg.header)
        for skeleton in skeleton_msg.skeletons:
            limb_to_pose = {}

            out_skeleton_msg = HumanSkeleton(header=skeleton_msg.header)
            for bone_name, bone in zip(skeleton.bone_names,
                                       skeleton.bones):
                u, v = bone.start_point.x, bone.start_point.y
                if 0 <= u < W and 0 <= v < H:
                    z = float(depth_img[int(v)][int(u)])
                else:
                    continue
                if np.isnan(z) or z <= 0:
                    continue
                start_x = (u - camera_model.cx()) * z / camera_model.fx()
                start_y = (v - camera_model.cy()) * z / camera_model.fy()
                start_z = z

                u, v = bone.end_point.x, bone.end_point.y
                if 0 <= u < W and 0 <= v < H:
                    z = float(depth_img[int(v)][int(u)])
                else:
                    continue
                if np.isnan(z) or z <= 0:
                    continue

                a, b = bone_name.split('->')
                limb_to_pose[a] = np.array(
                    [bone.start_point.x,
                     bone.start_point.y,
                     bone.start_point.z])
                limb_to_pose[b] = np.array(
                    [bone.end_point.x,
                     bone.end_point.y,
                     bone.end_point.z])

                bone.end_point.x = (u - camera_model.cx()
                                    ) * z / camera_model.fx()
                bone.end_point.y = (v - camera_model.cy()
                                    ) * z / camera_model.fy()
                bone.end_point.z = z

                bone.start_point.x = start_x
                bone.start_point.y = start_y
                bone.start_point.z = start_z
                out_skeleton_msg.bone_names.append(bone_name)
                out_skeleton_msg.bones.append(bone)
            out_skeleton_array_msg.skeletons.append(out_skeleton_msg)

        self.skeleton_pub.publish(out_skeleton_array_msg)


if __name__ == '__main__':
    rospy.init_node('skeleton_with_depth')
    SkeletonWithDepth()
    rospy.spin()
