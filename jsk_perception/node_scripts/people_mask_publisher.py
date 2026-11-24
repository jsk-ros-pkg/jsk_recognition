#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

import cv_bridge
import message_filters
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import Image


class PeopleMaskPublisher(ConnectionBasedTransport):

    limb_names = ["Nose",
                  "Neck",
                  "RShoulder",
                  "RElbow",
                  "RWrist",
                  "LShoulder",
                  "LElbow",
                  "LWrist",
                  "RHip",
                  "RKnee",
                  "RAnkle",
                  "LHip",
                  "LKnee",
                  "LAnkle",
                  "REye",
                  "LEye",
                  "REar",
                  "LEar",
                  'RHand',
                  'LHand']

    def __init__(self):
        super(self.__class__, self).__init__()
        self.person_indices = rospy.get_param('~person_indices', -1)
        self.limb_part = rospy.get_param('~limb_part', 'all')
        if self.limb_part == 'all':
            self.limb_part = self.limb_names
        self.arms_score_threshold = rospy.get_param(
            '~arms_score_threshold', 0.25)
        self.hand_ratio = rospy.get_param('~hand_ratio', 0.33)
        self.hand_width_ratio = rospy.get_param('~hand_width_ratio', 0.8)
        self.face_ratio = rospy.get_param('~face_ratio', 0.6)
        self.face_shoulder_ratio = rospy.get_param('~face_shoulder_ratio', 0.5)
        self.face_width_margin_ratio = rospy.get_param('~face_width_margin_ratio', 1.3)

        self.pub = self.advertise('~output', Image, queue_size=1)
        self.debug_pub = self.advertise('~debug/output', Image, queue_size=1)

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_img = message_filters.Subscriber(
            '~input', Image, queue_size=1, buff_size=2**24)
        sub_pose = message_filters.Subscriber(
            '~input/pose', PeoplePoseArray, queue_size=1, buff_size=2**24)
        self.subs = [sub_img, sub_pose]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb(self, img_msg, people_pose_array_msg):
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        if self.person_indices != -1:
            indices = np.array(self.person_indices)
            indices = indices[indices < len(people_pose_array_msg.poses)]
            people_pose = np.array(people_pose_array_msg.poses)[indices]
        else:
            people_pose = people_pose_array_msg.poses

        mask_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.bool)
        arm = [limb_prefix for limb_prefix in ['R', 'L']
               if limb_prefix + 'Hand' in self.limb_part]
        nose = 'Nose' in self.limb_part
        if arm:
            arm_mask_img, debug_img = self._create_hand_mask(people_pose, img, arm)
            mask_img = np.bitwise_or(mask_img, arm_mask_img)
        if nose:
            nose_mask_img, debug_img = self._create_nose_mask(people_pose, img)
            mask_img = np.bitwise_or(mask_img, nose_mask_img)

        mask_msg = br.cv2_to_imgmsg(np.uint8(mask_img * 255.0), encoding='mono8')
        mask_msg.header = img_msg.header

        debug_msg = br.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = img_msg.header

        self.pub.publish(mask_msg)
        self.debug_pub.publish(debug_msg)

    def _create_hand_mask(self, people_pose, img, arm=['R', 'L']):
        rectangle_thickness = 5
        rectangle_colors = [(0, 255, 0), (0, 0, 255)]

        mask_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.bool)
        for person_pose in people_pose:
            for limb_prefix, color in zip(arm, rectangle_colors):
                try:
                    shoulder_index = person_pose.limb_names.index(
                        limb_prefix + 'Shoulder')
                    elbow_index = person_pose.limb_names.index(
                        limb_prefix + 'Elbow')
                    wrist_index = person_pose.limb_names.index(
                        limb_prefix + 'Wrist')
                except ValueError:
                    continue

                if not np.all(np.array(person_pose.scores)[[elbow_index, shoulder_index, wrist_index]] > self.arms_score_threshold):
                    continue

                shoulder = person_pose.poses[shoulder_index]
                elbow = person_pose.poses[elbow_index]
                wrist = person_pose.poses[wrist_index]

                x = wrist.position.x + self.hand_ratio * \
                    (wrist.position.x - elbow.position.x)
                y = wrist.position.y + self.hand_ratio * \
                    (wrist.position.y - elbow.position.y)

                wrist_to_elbow_length = ((wrist.position.x - elbow.position.x)
                                         ** 2 + (wrist.position.y - elbow.position.y) ** 2) ** 0.5
                elbow_to_shoulder_length = ((shoulder.position.x - elbow.position.x) ** 2 + (
                    shoulder.position.y - elbow.position.y) ** 2) ** 0.5
                width = self.hand_width_ratio * \
                    max(wrist_to_elbow_length, 0.9 * elbow_to_shoulder_length)
                height = width

                x -= width / 2.0
                y -= height / 2.0

                x = int(x)
                y = int(y)
                width = int(width)
                height = int(height)

                mask_img[max(y, 0):max(min(y + height, img.shape[0]), 0),
                         max(x, 0):max(min(x + width, img.shape[1]), 0)] = True
                cv2.rectangle(img, (x, y), (x + width, y + height),
                              color, rectangle_thickness)
        return mask_img, img

    def _create_nose_mask(self, people_pose, img):
        rectangle_thickness = 5
        color = (0, 255, 0)

        mask_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.bool)
        for person_pose in people_pose:
            x_max = -10000
            x_min = 10000

            limb_poses = []
            nose_pose = None
            neck_pose = None
            target_limb_names = ['Nose',
                                 'Neck',
                                 'REye',
                                 'LEye',
                                 'REar',
                                 'LEar']
            for limb_name in target_limb_names:
                try:
                    limb_index = person_pose.limb_names.index(limb_name)
                    limb_pose = person_pose.poses[limb_index]
                    limb_poses.append(limb_pose)
                    if limb_name == 'Nose':
                        nose_pose = limb_pose
                    elif limb_name == 'Neck':
                        neck_pose = limb_pose
                except ValueError:
                    continue

            shoulder_limb_poses = []
            shoulder_limb_names = ['RShoulder',
                                   'LShoulder']
            for limb_name in shoulder_limb_names:
                try:
                    shoulder_limb_index = person_pose.limb_names.index(limb_name)
                    shoulder_limb_pose = person_pose.poses[shoulder_limb_index]
                    shoulder_limb_poses.append(shoulder_limb_pose)
                except ValueError:
                    continue
            if len(shoulder_limb_poses) == 2:
                positions_x = [pose.position.x for pose in shoulder_limb_poses]
                x_mid = sum(positions_x) / len(positions_x)
                x_max = max(positions_x) * self.face_shoulder_ratio + \
                        x_mid * (1.0 - self.face_shoulder_ratio)
                x_min = min(positions_x) * self.face_shoulder_ratio + \
                        x_mid * (1.0 - self.face_shoulder_ratio)

            if nose_pose == None or neck_pose == None:
                continue

            positions_x = [pose.position.x for pose in limb_poses]
            x_max = max(max(positions_x), x_max)
            x_min = min(min(positions_x), x_min)
            width = (x_max - x_min) * self.face_width_margin_ratio
            x = (x_max + x_min) / 2.0 - width / 2.0

            positions_y = [pose.position.y for pose in limb_poses]
            y_min = min(positions_y)
            height = self.face_ratio * \
                     abs(nose_pose.position.y - neck_pose.position.y) * 2.0
            y_min = min(y_min, nose_pose.position.y - height / 2.0)
            y_max = nose_pose.position.y + height / 2.0
            y = y_min
            height = y_max - y_min

            x = int(x)
            y = int(y)
            width = int(width)
            height = int(height)

            mask_img[max(y, 0):max(min(y + height, img.shape[0]), 0),
                     max(x, 0):max(min(x + width, img.shape[1]), 0)] = True
            cv2.rectangle(img, (x, y), (x + width, y + height),
                          color, rectangle_thickness)
        return mask_img, img

if __name__ == '__main__':
    rospy.init_node('people_mask_publisher')
    PeopleMaskPublisher()
    rospy.spin()
