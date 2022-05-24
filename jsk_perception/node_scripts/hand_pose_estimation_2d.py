#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Reference
#   SRHandNet: Real-time 2D Hand Pose Estimation
#              with Simultaneous Region Localization
# SRHandNet model downloading
#   https://www.yangangwang.com/papers/WANG-SRH-2019-07.html
#
# Code Reference
#   https://github.com/agrajak/SRHandNet-demo
from __future__ import division
from __future__ import print_function

from distutils.version import LooseVersion
import pkg_resources
import sys

if LooseVersion(pkg_resources.get_distribution("torch").version) \
       < LooseVersion('1.1.0'):
    print('''torch >= 1.1.0 is required:
    Download wheel file from https://download.pytorch.org/whl/cu90/torch_stable.html (melodic)
    and install with pip as follow:
    pip install --user torch-1.1.0-cp27-cp27mu-linux_x86_64.whl
    pip install --user torchvision-0.3.0-cp27-cp27mu-manylinux1_x86_64.whl

''', file=sys.stderr)
    sys.exit(1)

import cv2
import cv_bridge
import message_filters
import jsk_data
import numpy as np
import os.path as osp
import rospkg
import rospy
from skimage import feature
import torch

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HandPose
from jsk_recognition_msgs.msg import HandPoseArray
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import Segment
from jsk_topic_tools import ConnectionBasedTransport
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class HandPoseEstimation2D(ConnectionBasedTransport):

    HAND_COLORS = [
        (100, 100, 100),
        (100, 0, 0),
        (150, 0, 0),
        (200, 0, 0),
        (255, 0, 0),
        (100, 100, 0),
        (150, 150, 0),
        (200, 200, 0),
        (255, 255, 0),
        (0, 100, 50),
        (0, 150, 75),
        (0, 200, 100),
        (0, 255, 125),
        (0, 50, 100),
        (0, 75, 150),
        (0, 100, 200),
        (0, 125, 255),
        (100, 0, 100),
        (150, 0, 150),
        (200, 0, 200),
        (255, 0, 255),
    ]
    # The order of keypoints is predefined in SRHandNet model
    # Following the order of cooresponding 21-channel feature maps
    INDEX2FINGERNAME = [
        "wrist",
        "thumb_mcp",
        "thumb_pip",
        "thumb_dip",
        "thumb_tip",
        "index_mcp",
        "index_pip",
        "index_dip",
        "index_tip",
        "middle_mcp",
        "middle_pip",
        "middle_dip",
        "middle_tip",
        "ring_mcp",
        "ring_pip",
        "ring_dip",
        "ring_tip",
        "little_mcp",
        "little_pip",
        "little_dip",
        "little_tip",
    ]

    FINGERNAME2INDEX = {name: i for i, name in enumerate(INDEX2FINGERNAME)}
    NUMBER_OF_FINGERS = 5
    NUMBER_OF_FINGER_JOINT = 3
    CONNECTION_PAIR = [(0, 1), (0, 1), (1, 2), (2, 3),
                       (0, 5), (4, 5), (5, 6), (6, 7),
                       (0, 9), (8, 9), (9, 10), (10, 11),
                       (0, 13), (12, 13), (13, 14), (14, 15),
                       (0, 17), (16, 17), (17, 18), (18, 19)]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.backend = rospy.get_param('~backend', 'torch')
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        # default tensor size
        self.TRAIN_IMAGE_HEIGHT = 256
        self.TRAIN_IMAGE_WIDTH = 256
        self.label_bbox_min = rospy.get_param('~thre1', 0.3)
        self.label_hand_min = rospy.get_param('~thre2', 0.2)
        # detection fail if more than ~thre3 keypoints missed
        self.missing_point = rospy.get_param('~thre3', 5)
        # model loading
        self._load_model()
        # image subscribe
        # self.subscribe()
        # topic advertise
        self.image_pub = self.advertise(
            '~output/vis', Image, queue_size=1)
        self.hand_pose_pub = self.advertise(
            '~output/pose', HandPoseArray, queue_size=1)
        self.with_depth = rospy.get_param("~with_depth", False)
        if self.with_depth is True:
            self.hand_pose_2d_pub = self.advertise(
            '~output/pose_2d', HandPoseArray, queue_size=1)
            self.skeleton_pub = self.advertise(
                '~skeleton', HumanSkeletonArray, queue_size=1)
        self.bridge = cv_bridge.CvBridge()

    @property
    def visualize(self):
        return self.image_pub.get_num_connections() > 0

    def _load_model(self):
        if self.backend == 'torch':
            self._load_torch_model()
        else:
            raise RuntimeError('Unsupported backend: %s', self.backend)

    def _load_torch_model(self):
        # model can be downloaded manually from:
        #    https://www.yangangwang.com/papers/WANG-SRH-2019-07.html
        # or using _get_srhand_pretrained_model() from:
        #    https://drive.google.com/uc?id=16Jg8HhaFaThzFSbWbEixMLE3SAnOzvzL
        rospy.loginfo('Loading model')
        model_file = rospy.get_param('~model_file', None)
        if not model_file:
            model_file = self._get_srhand_pretrained_model()

        if self.gpu >= 0 and torch.cuda.is_available():
            self.model = torch.jit.load(
                model_file, map_location=torch.device('cuda'))
            rospy.loginfo('Finished loading SRHandNet model to gpu')
        else:
            self.model = torch.jit.load(
                model_file, map_location=torch.device('cpu'))
            rospy.loginfo('Finished loading SRHandNet model to cpu')
            if self.gpu >= 0:
                rospy.loginfo('You need to check version match between PyTorch and CUDA. See https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/install_chainer_gpu.html and https://github.com/jsk-ros-pkg/jsk_recognition/pull/2601#issuecomment-876405219 for more info')
        self.model.eval()

    def _get_srhand_pretrained_model(self):
        pkg_name = 'jsk_perception'
        filepath = 'trained_data/SRHandNet.pts'
        jsk_data.download_data(
            pkg_name=pkg_name,
            path=filepath,
            url='https://drive.google.com/'
            'uc?id=10oGfGALjIIwdIWO9MQftN07A3TRdvp3b',
            md5='9f39d3baa43cf1c962c8f752c009eb14',
        )
        rp = rospkg.RosPack()
        pkgpath = rp.get_path(pkg_name)
        return osp.join(pkgpath, filepath)

    def subscribe(self):
        if self.with_depth:
            queue_size = rospy.get_param('~queue_size', 10)
            sub_img = message_filters.Subscriber(
                '~input', Image, queue_size=1, buff_size=2**24)
            sub_depth = message_filters.Subscriber(
                '~input/depth', Image, queue_size=1, buff_size=2**24)
            self.subs = [sub_img, sub_depth]

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
        else:
            sub_img = rospy.Subscriber(
                '~input', Image, self._cb, queue_size=1, buff_size=2**24)
            self.subs = [sub_img]

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

    def _cb_with_depth(self, img_msg, depth_msg):
        if self.camera_info_msg is None:
            return
        self._cb_with_depth_info(img_msg, depth_msg, self.camera_info_msg)

    def _cb_with_depth_info(self, img_msg, depth_msg, camera_info_msg):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_msg)
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        hands_points, hands_point_scores, hands_score = self.hand_pose_estimate(img)

        hand_pose_2d_msg = self._create_2d_hand_pose_array_msgs(
            hands_points,
            hands_point_scores,
            hands_score,
            img_msg.header)

        # calculate xyz-position
        hand_pose_array_msg = HandPoseArray(header=img_msg.header)

        for hand in hand_pose_2d_msg.poses:
            hand_pose_msg = HandPose(hand_score=hand.hand_score)
            for pose, finger_name, score in zip(
                    hand.poses, hand.finger_names,
                    hand.point_scores):
                u = pose.position.x
                v = pose.position.y
                if 0 <= u < depth_img.shape[1] and \
                   0 <= v < depth_img.shape[0]:
                    z = float(depth_img[int(v)][int(u)])
                else:
                    continue
                if np.isnan(z):
                    continue
                x = (u - camera_model.cx()) * z / camera_model.fx()
                y = (v - camera_model.cy()) * z / camera_model.fy()
                hand_pose_msg.finger_names.append(finger_name)
                hand_pose_msg.poses.append(
                    Pose(position=Point(x=x, y=y, z=z),
                         orientation=Quaternion(w=1)))
                hand_pose_msg.point_scores.append(score)
            hand_pose_array_msg.poses.append(hand_pose_msg)

        # prepare HumanSkeletonArray for visualization.
        skeleton_array_msg = HumanSkeletonArray(header=img_msg.header)
        for hand_pose_msg in hand_pose_array_msg.poses:
            hand_joint_index_to_list_index = {
                self.FINGERNAME2INDEX[name]: i
                for i, name in enumerate(hand_pose_msg.finger_names)}
            skeleton_msg = HumanSkeleton(header=img_msg.header)
            for index_a, index_b in self.CONNECTION_PAIR:
                if index_a not in hand_joint_index_to_list_index \
                   or index_b not in hand_joint_index_to_list_index:
                    continue
                joint_a_index = hand_joint_index_to_list_index[index_a]
                joint_b_index = hand_joint_index_to_list_index[index_b]
                bone = Segment(
                    start_point=hand_pose_msg.poses[joint_a_index].position,
                    end_point=hand_pose_msg.poses[joint_b_index].position)
                bone_name = '{}->{}'.format(
                    self.INDEX2FINGERNAME[index_a],
                    self.INDEX2FINGERNAME[index_b])
                skeleton_msg.bones.append(bone)
                skeleton_msg.bone_names.append(bone_name)
            skeleton_array_msg.skeletons.append(skeleton_msg)

        self.hand_pose_2d_pub.publish(hand_pose_2d_msg)
        self.hand_pose_pub.publish(hand_pose_array_msg)
        self.skeleton_pub.publish(skeleton_array_msg)

    def _cb(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8')
        hands_points, hands_point_scores, hands_score = \
            self.hand_pose_estimate(img)

        hand_pose_msg = self._create_2d_hand_pose_array_msgs(
            hands_points,
            hands_point_scores,
            hands_score,
            img_msg.header)

        self.hand_pose_pub.publish(hand_pose_msg)

    # create hand_pose_msg consists of
    #        hand_score got from bounding box feature map
    #        21 keypoint positions(set to 0,0,0 if not detected)
    #        corresponding joint names
    #        corresponding scores got from feature map
    # for each detected hand
    def _create_2d_hand_pose_array_msgs(
            self, hands_points,
            hands_point_scores,
            hands_score,
            header
    ):
        hand_pose_msg = HandPoseArray(header=header)
        for hand_points, point_scores, hand_score in \
                zip(hands_points, hands_point_scores, hands_score):
            pose_msg = HandPose()
            pose_msg.hand_score = hand_score
            for index, joint_pos in enumerate(hand_points):
                pose_msg.finger_names.append(self.INDEX2FINGERNAME[index])
                pose_msg.point_scores.append(point_scores[index])
                if (len(joint_pos) == 2):
                    pose_msg.poses.append(
                        Pose(position=Point(
                            x=joint_pos[0], y=joint_pos[1], z=0.)))
                else:
                    pose_msg.poses.append(
                        Pose(position=Point(x=0., y=0., z=0.)))
            hand_pose_msg.poses.append(pose_msg)
        return hand_pose_msg

    def hand_pose_estimate(self, bgr):
        if self.backend == 'torch':
            return self._hand_pose_estimate_torch_backend(bgr)
        raise ValueError('Unsupported backend: {0}'.format(self.backend))

    def _hand_pose_estimate_torch_backend(self, frame):
        hands_points, hands_rect, hands_point_scores, hands_score = \
            self.pyramid_inference(frame)

        if self.visualize:
            vis_img = self._draw_joints(frame, hands_points, hands_rect)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
            self.image_pub.publish(vis_msg)

        return hands_points, hands_point_scores, hands_score

    def _draw_joints(self, frame, hands_points, hands_rect):
        for rect_idx, points in enumerate(hands_points):
            rect = hands_rect[rect_idx]
            if rect is None:
                continue
            # bounding boxes
            cv2.rectangle(frame, rect[0:2], rect[2:4], (0, 0, 255), 6)
            missing = 0

            for i, point in enumerate(points):
                if point is None or len(point) == 0:
                    missing += 1
                    continue
                # joint keypoints
                cv2.circle(frame, point, 6, self.HAND_COLORS[i], 6)

            for i in range(5):
                for j in range(3):
                    cnt = j+i*4+1
                    if (len(points[cnt]) != 0
                            and len(points[cnt+1]) != 0):
                        # fingers
                        cv2.line(
                            frame, points[cnt], points[cnt+1],
                            (0, 255, 0), 2)

            per = '{}%'.format(int(2100-missing*100)//21)
            text_pos = hands_rect[rect_idx][0:2]
            text_pos = (text_pos[0], text_pos[1]+5)
            cv2.putText(frame, per, text_pos, 1, 3, (0, 0, 255), 3)
        return frame

    # finding local maximum
    def nmslocation(self, src, threshold):
        peak_loc = feature.peak_local_max(
            src, min_distance=2, threshold_abs=threshold, exclude_border=True)
        peak_pair = [(src[x][y], (x, y)) for x, y in peak_loc]
        return peak_pair

    # resize src_img to fit the size of tensor
    def transform_net_input(
            self, tensor, src_img, hands_rect=None, tensor_idx=0):
        img = src_img.copy()
        if hands_rect is not None:
            l, t, r, b = hands_rect[tensor_idx]
            img = img[t:b, l:r]

        rows, cols = img.shape[:2]
        ratio = min(tensor.shape[2] / rows, tensor.shape[3] / cols)
        mat = np.array([[ratio, 0, 0], [0, ratio, 0]], dtype=np.float32)

        dst = cv2.warpAffine(
            img, mat, (tensor.shape[3], tensor.shape[2]),
            flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
            borderValue=(128, 128, 128))

        dst = dst / 255. - 0.5
        b, g, r = cv2.split(dst)

        if self.gpu >= 0 and torch.cuda.is_available():
            tensor[tensor_idx][0] = torch.tensor(
                b, device=torch.device('cuda')).float()
            tensor[tensor_idx][1] = torch.tensor(
                g, device=torch.device('cuda')).float()
            tensor[tensor_idx][2] = torch.tensor(
                r, device=torch.device('cuda')).float()
        else:
            tensor[tensor_idx][0] = torch.tensor(
                b, device=torch.device('cpu')).float()
            tensor[tensor_idx][1] = torch.tensor(
                g, device=torch.device('cpu')).float()
            tensor[tensor_idx][2] = torch.tensor(
                r, device=torch.device('cpu')).float()
        return ratio

    def detect_bbox(self, input_image):
        if self.gpu >= 0 and torch.cuda.is_available():
            tensor = torch.zeros(
                [1, 3, self.TRAIN_IMAGE_HEIGHT, self.TRAIN_IMAGE_WIDTH],
                device=torch.device('cuda'))
        else:
            tensor = torch.zeros(
                [1, 3, self.TRAIN_IMAGE_HEIGHT, self.TRAIN_IMAGE_WIDTH],
                device=torch.device('cpu'))
        rows, cols, _ = input_image.shape
        # transform the input data
        ratio_input_to_net = self.transform_net_input(tensor, input_image)

        with torch.no_grad():
            heatmap = self.model.forward(tensor)[3]

        # copy the 3-channel rectmap, each channel respectively representing:
        #     centre position of bounding box
        #     width of bounding box
        #     height of bounding box
        ratio_net_downsample = \
            self.TRAIN_IMAGE_HEIGHT / float(heatmap.shape[2])
        rect_map_idx = heatmap.shape[1] - 3
        rectmap = []
        for i in range(3):
            rectmap.append(
                np.copy(heatmap[0][i+rect_map_idx].cpu().detach().numpy()))

        # centre position of bounding boxes
        locations = self.nmslocation(rectmap[0], self.label_bbox_min)
        hands_rect = []
        hands_score = []
        for loc_val, points in locations:
            pos_x, pos_y = points
            ratio_width = ratio_height = pixelcount = 0
            for m in range(
                    max(pos_x-2, 0), min(pos_x+3, int(heatmap.shape[2]))):
                for n in range(
                        max(pos_y-2, 0), min(pos_y+3, int(heatmap.shape[3]))):
                    ratio_width += rectmap[1][m][n]
                    ratio_height += rectmap[2][m][n]
                    pixelcount += 1

            if pixelcount > 0:
                ratio_width = min(max(ratio_width / pixelcount, 0), 1)
                ratio_height = min(max(ratio_height / pixelcount, 0), 1)
                ratio = ratio_net_downsample / ratio_input_to_net
                pos_x *= ratio  # row
                pos_y *= ratio  # column

                rect_w = \
                    ratio_width * self.TRAIN_IMAGE_WIDTH / ratio_input_to_net
                rect_h = \
                    ratio_height * self.TRAIN_IMAGE_HEIGHT / ratio_input_to_net
                # left-top corner position
                l_t = (
                    max(int(pos_x - rect_h/2), 0),
                    max(int(pos_y - rect_w/2), 0)
                )
                # right-bottom corner position
                r_b = (
                    min(int(pos_x + rect_h/2), rows - 1),
                    min(int(pos_y + rect_w/2), cols - 1)
                )

                hands_rect.append((l_t[1], l_t[0], r_b[1], r_b[0]))
                hands_score.append(loc_val)
        return hands_rect, hands_score

    def detect_hand(self, input_image, hands_rect):
        if len(hands_rect) == 0:
            return [], []

        ratio_input_to_net = [None]*len(hands_rect)

        if self.gpu >= 0 and torch.cuda.is_available():
            tensor = torch.zeros(
                [len(hands_rect), 3,
                 self.TRAIN_IMAGE_HEIGHT, self.TRAIN_IMAGE_WIDTH],
                device=torch.device('cuda'))
        else:
            tensor = torch.zeros(
                [len(hands_rect), 3,
                 self.TRAIN_IMAGE_HEIGHT, self.TRAIN_IMAGE_WIDTH],
                device=torch.device('cpu'))

        # extract RoI of the input_image and resize to fit the tensor size
        for i in range(len(hands_rect)):
            ratio_input_to_net[i] = self.transform_net_input(
                tensor, input_image, hands_rect, i)

        with torch.no_grad():
            heatmaps = self.model.forward(tensor)[3]

        ratio_net_downsample = self.TRAIN_IMAGE_HEIGHT / heatmaps.size()[2]

        # joint position
        hands_points = []
        hands_point_scores = []
        for rect_idx in range(len(hands_rect)):
            hand_points = [[] for i in range(21)]
            point_scores = [0. for i in range(21)]
            x, y, _, _ = hands_rect[rect_idx]
            ratio = ratio_net_downsample / ratio_input_to_net[rect_idx]
            for i in range(21):
                heatmap = heatmaps[rect_idx][i].cpu().detach().numpy()
                points = self.nmslocation(heatmap, self.label_hand_min)
                if len(points):
                    score, point = points[0]
                    hand_points[i] = (
                        int(point[1]*ratio)+x,
                        int(point[0]*ratio)+y
                    )
                    point_scores[i] = score
            hands_points.append(hand_points)
            hands_point_scores.append(point_scores)
        return hands_points, hands_point_scores

    def pyramid_inference(self, input_image):
        # Full Cycle Detection
        # Region of Interest
        hands_rect, hands_score = self.detect_bbox(input_image)

        if len(hands_rect) == 0:
            return [], [], [], []

        # joints detection
        hands_points, hands_point_scores = self.detect_hand(
            input_image, hands_rect)

        for i in range(len(hands_rect)-1, -1, -1):
            missing_points = 0
            for j in range(21):
                if len(hands_points[i][j]) != 2:
                    missing_points += 1
            if missing_points > self.missing_point:
                hands_rect.pop(i)
                hands_score.pop(i)
                hands_points.pop(i)
                hands_point_scores.pop(i)

        return hands_points, hands_rect, hands_point_scores, hands_score


if __name__ == '__main__':
    rospy.init_node('hand_pose_estimation_2d')
    HandPoseEstimation2D()
    rospy.spin()
