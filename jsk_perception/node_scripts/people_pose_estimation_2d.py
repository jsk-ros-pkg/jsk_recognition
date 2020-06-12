#!/usr/bin/env python
# -*- coding:utf-8 -*-

#    See: https://arxiv.org/abs/1611.08050

from __future__ import print_function

import math

import itertools, pkg_resources, sys
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
   sys.version_info.major == 2:
   print('''Please install chainer <= 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
   sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name ] == []:
   print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
   sys.exit(1)
import chainer
import chainer.functions as F
from chainer import cuda
import cv2
import matplotlib
import numpy as np
from scipy.ndimage.filters import gaussian_filter
import pylab as plt  # NOQA

import cv_bridge
import message_filters
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_recognition_msgs.msg import Segment
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from openpose import PoseNet, HandNet


def find_joint(limb, jts):
    jt = [jt for jt in jts if jt['limb'] == limb]
    if jt:
        return jt[0]
    else:
        return None


def padRightDownCorner(img, stride, padValue):
    h = img.shape[0]
    w = img.shape[1]

    pad = 4 * [None]
    pad[0] = 0  # up
    pad[1] = 0  # left
    pad[2] = 0 if (h % stride == 0) else stride - (h % stride)  # down
    pad[3] = 0 if (w % stride == 0) else stride - (w % stride)  # right

    img_padded = img
    pad_up = np.tile(img_padded[0:1, :, :] * 0 + padValue, (pad[0], 1, 1))
    img_padded = np.concatenate((pad_up, img_padded), axis=0)
    pad_left = np.tile(img_padded[:, 0:1, :] * 0 + padValue, (1, pad[1], 1))
    img_padded = np.concatenate((pad_left, img_padded), axis=1)
    pad_down = np.tile(img_padded[-2:-1, :, :] * 0 + padValue, (pad[2], 1, 1))
    img_padded = np.concatenate((img_padded, pad_down), axis=0)
    pad_right = np.tile(img_padded[:, -2:-1, :] * 0 + padValue, (1, pad[3], 1))
    img_padded = np.concatenate((img_padded, pad_right), axis=1)

    return img_padded, pad


class PeoplePoseEstimation2D(ConnectionBasedTransport):
    # Note: the order of this sequences is important
    # find connection in the specified sequence,
    # center 29 is in the position 15
    limb_sequence = [[ 2,  1], [ 1, 16], [ 1, 15], [ 6, 18], [ 3, 17],
                     [ 2,  3], [ 2,  6], [ 3,  4], [ 4,  5], [ 6,  7],
                     [ 7,  8], [ 2,  9], [ 9, 10], [10, 11], [ 2, 12],
                     [12, 13], [13, 14], [15, 17], [16, 18]]
    # the middle joints heatmap correpondence
    map_idx = [[47, 48], [49, 50], [51, 52], [37, 38], [45, 46],
               [31, 32], [39, 40], [33, 34], [35, 36], [41, 42],
               [43, 44], [19, 20], [21, 22], [23, 24], [25, 26],
               [27, 28], [29, 30], [53, 54], [55, 56]]
    # length ratio from connections
    limb_length_hand_ratio = [ 0.6,  0.2,  0.2, 0.85, 0.85,
                               0.6,  0.6, 0.93, 0.65, 0.95,
                              0.65,  2.2,  1.7,  1.7,  2.2,
                               1.7,  1.7, 0.25, 0.25]
    # hand joint connection sequence
    hand_sequence = [[0, 1],   [1, 2],   [2, 3],   [3, 4],
                     [0, 5],   [5, 6],   [6, 7],   [7, 8],
                     [0, 9],   [9, 10],  [10, 11], [11, 12],
                     [0, 13],  [13, 14], [14, 15], [15, 16],
                     [0, 17],  [17, 18], [18, 19], [19, 20],]

    index2limbname = ["Nose",
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
                      "Bkg"]

    index2handname = ["RHand{}".format(i) for i in range(21)] +\
                     ["LHand{}".format(i) for i in range(21)]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.backend = rospy.get_param('~backend', 'chainer')
        self.scales = rospy.get_param('~scales', [0.38])
        self.stride = rospy.get_param('~stride', 8)
        self.pad_value = rospy.get_param('~pad_value', 128)
        self.thre1 = rospy.get_param('~thre1', 0.1)
        self.thre2 = rospy.get_param('~thre2', 0.05)
        self.width = rospy.get_param('~width', None)
        self.height = rospy.get_param('~height', None)
        self.check_wh()
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        self.with_depth = rospy.get_param('~with_depth', False)
        # hand detection
        self.use_hand = rospy.get_param('~hand/enable', False)
        self.hand_gaussian_ksize = rospy.get_param('~hand/gaussian_ksize', 17)
        self.hand_gaussian_sigma = rospy.get_param('~hand/gaussian_sigma', 2.5)
        self.hand_thre1 = rospy.get_param('~hand/thre1', 20)
        self.hand_thre2 = rospy.get_param('~hand/thre2', 0.1)
        self.hand_width_offset = rospy.get_param('~hand/width_offset', 0)
        # model loading
        self._load_model()
        # topic advertise
        self.image_pub = self.advertise('~output', Image, queue_size=1)
        self.pose_pub = self.advertise('~pose', PeoplePoseArray, queue_size=1)
        self.sub_info = None
        if self.with_depth is True:
            self.pose_2d_pub = self.advertise('~pose_2d', PeoplePoseArray, queue_size=1)
            # visualization rviz plugin: https://github.com/jsk-ros-pkg/jsk_visualization/pull/740
            self.skeleton_pub = self.advertise(
                '~skeleton', HumanSkeletonArray, queue_size=1)

    def check_wh(self):
        if (self.width is None) != (self.height is None):
            rospy.logwarn('width and height should be specified, but '
                          'specified only {}'
                          .format('height' if self.height else 'width'))

    @property
    def visualize(self):
        return self.image_pub.get_num_connections() > 0

    def _load_model(self):
        if self.backend == 'chainer':
            self._load_chainer_model()
        else:
            raise RuntimeError('Unsupported backend: %s', self.backend)

    def _load_chainer_model(self):
        model_file = rospy.get_param('~model_file')
        self.pose_net = PoseNet(pretrained_model=model_file)
        rospy.loginfo('Finished loading trained model: {0}'.format(model_file))
        # hand net
        if self.use_hand:
            model_file = rospy.get_param('~hand/model_file')
            self.hand_net = HandNet(pretrained_model=model_file)
            rospy.loginfo('Finished loading trained hand model: {}'.format(model_file))
        #
        if self.gpu >= 0:
            self.pose_net.to_gpu(self.gpu)
            if self.use_hand:
                self.hand_net.to_gpu(self.gpu)
                # create gaussian kernel
                ksize = self.hand_gaussian_ksize
                sigma = self.hand_gaussian_sigma
                c = ksize // 2
                k = np.zeros((1, 1, ksize, ksize), dtype=np.float32)
                for y in range(ksize):
                    dy = abs(y - c)
                    for x in range(ksize):
                        dx = abs(x - c)
                        e = np.exp(- (dx ** 2 + dy ** 2) / (2 * sigma ** 2))
                        k[0][0][y][x] = 1 / (sigma ** 2 * 2 * np.pi) * e
                k = chainer.cuda.to_gpu(k, device=self.gpu)
                self.hand_gaussian_kernel = k
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False

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
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth_img = br.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth_img = np.asarray(depth_img, dtype=np.float32)
            depth_img /= 1000  # convert metric: mm -> m
        elif depth_msg.encoding != '32FC1':
            rospy.logerr('Unsupported depth encoding: %s' % depth_msg.encoding)

        people_joint_positions, all_peaks = self.pose_estimate(img)
        if self.use_hand:
            people_joint_positions = self.hand_estimate(
                img, people_joint_positions)

        people_pose_msg = PeoplePoseArray()
        people_pose_msg.header = img_msg.header
        people_pose_2d_msg = self._create_2d_people_pose_array_msgs(
            people_joint_positions,
            img_msg.header)
        skeleton_msgs = HumanSkeletonArray(header=img_msg.header)

        # calculate xyz-position
        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]
        for person_joint_positions in people_joint_positions:
            pose_msg = PeoplePose()
            skeleton_msg = HumanSkeleton(header=img_msg.header)
            for joint_pos in person_joint_positions:
                if joint_pos['score'] < 0:
                    continue
                if 0 <= joint_pos['y'] < depth_img.shape[0] and\
                   0 <= joint_pos['x'] < depth_img.shape[1]:
                    z = float(depth_img[int(joint_pos['y'])][int(joint_pos['x'])])
                else:
                    continue
                if np.isnan(z):
                    continue
                x = (joint_pos['x'] - cx) * z / fx
                y = (joint_pos['y'] - cy) * z / fy
                pose_msg.limb_names.append(joint_pos['limb'])
                pose_msg.scores.append(joint_pos['score'])
                pose_msg.poses.append(Pose(position=Point(x=x, y=y, z=z),
                                           orientation=Quaternion(w=1)))
            people_pose_msg.poses.append(pose_msg)

            for i, conn in enumerate(self.limb_sequence):
                j1_name = self.index2limbname[conn[0] - 1]
                j2_name = self.index2limbname[conn[1] - 1]
                if j1_name not in pose_msg.limb_names \
                        or j2_name not in pose_msg.limb_names:
                    continue
                j1_index = pose_msg.limb_names.index(j1_name)
                j2_index = pose_msg.limb_names.index(j2_name)
                bone_name = '{}->{}'.format(j1_name, j2_name)
                bone = Segment(
                    start_point=pose_msg.poses[j1_index].position,
                    end_point=pose_msg.poses[j2_index].position)
                skeleton_msg.bones.append(bone)
                skeleton_msg.bone_names.append(bone_name)
            skeleton_msgs.skeletons.append(skeleton_msg)

        self.pose_2d_pub.publish(people_pose_2d_msg)
        self.pose_pub.publish(people_pose_msg)
        self.skeleton_pub.publish(skeleton_msgs)

        if self.visualize:
            vis_img = self._draw_joints(img, people_joint_positions, all_peaks)
            vis_msg = br.cv2_to_imgmsg(vis_img, encoding='bgr8')
            vis_msg.header.stamp = img_msg.header.stamp
            self.image_pub.publish(vis_msg)

    def _cb(self, img_msg):
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        people_joint_positions, all_peaks = self.pose_estimate(img)
        if self.use_hand:
            people_joint_positions = self.hand_estimate(
                img, people_joint_positions)

        people_pose_msg = self._create_2d_people_pose_array_msgs(
            people_joint_positions,
            img_msg.header)

        self.pose_pub.publish(people_pose_msg)

        if self.visualize:
            vis_img = self._draw_joints(img, people_joint_positions, all_peaks)
            vis_msg = br.cv2_to_imgmsg(vis_img, encoding='bgr8')
            vis_msg.header.stamp = img_msg.header.stamp
            self.image_pub.publish(vis_msg)

    def _create_2d_people_pose_array_msgs(self, people_joint_positions, header):
        people_pose_msg = PeoplePoseArray(header=header)
        for person_joint_positions in people_joint_positions:
            pose_msg = PeoplePose()
            for joint_pos in person_joint_positions:
                if joint_pos['score'] < 0:
                    continue
                pose_msg.limb_names.append(joint_pos['limb'])
                pose_msg.scores.append(joint_pos['score'])
                pose_msg.poses.append(Pose(position=Point(x=joint_pos['x'],
                                                          y=joint_pos['y'],
                                                          z=0)))
            people_pose_msg.poses.append(pose_msg)
        return people_pose_msg

    def pose_estimate(self, bgr):
        if self.backend == 'chainer':
            return self._pose_estimate_chainer_backend(bgr)
        raise ValueError('Unsupported backend: {0}'.format(self.backend))

    def _pose_estimate_chainer_backend(self, bgr_img):
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        xp = self.pose_net.xp

        org_h, org_w, _ = bgr_img.shape
        if not (self.width is None or self.height is None):
            bgr_img = cv2.resize(bgr_img, (self.width, self.height))

        heatmap_avg = xp.zeros((bgr_img.shape[0], bgr_img.shape[1], 19),
                               dtype=np.float32)
        paf_avg = xp.zeros((bgr_img.shape[0], bgr_img.shape[1], 38),
                           dtype=np.float32)

        for scale in self.scales:
            img = cv2.resize(bgr_img, (0, 0), fx=scale,
                             fy=scale, interpolation=cv2.INTER_CUBIC)
            padded_img, pad = padRightDownCorner(
                img, self.stride, self.pad_value)
            x = np.transpose(np.float32(
                padded_img[:, :, :, np.newaxis]), (3, 2, 0, 1)) / 256 - 0.5
            if self.gpu >= 0:
                x = chainer.cuda.to_gpu(x)
            x = chainer.Variable(x)
            pafs, heatmaps = self.pose_net(x)
            paf = pafs[-1]
            heatmap = heatmaps[-1]

            # extract outputs, resize, and remove padding
            heatmap = F.resize_images(
                heatmap, (heatmap.data.shape[2] * self.stride,
                          heatmap.data.shape[3] * self.stride))
            heatmap = heatmap[:, :, :padded_img.shape[0] -
                              pad[2], :padded_img.shape[1] - pad[3]]
            heatmap = F.resize_images(
                heatmap, (bgr_img.shape[0], bgr_img.shape[1]))
            heatmap = xp.transpose(xp.squeeze(heatmap.data), (1, 2, 0))
            paf = F.resize_images(
                paf, (paf.data.shape[2] * self.stride,
                      paf.data.shape[3] * self.stride))
            paf = paf[:, :, :padded_img.shape[0] -
                      pad[2], :padded_img.shape[1] - pad[3]]
            paf = F.resize_images(paf, (bgr_img.shape[0], bgr_img.shape[1]))
            paf = xp.transpose(xp.squeeze(paf.data), (1, 2, 0))

            coeff = 1.0 / len(self.scales)
            paf_avg += paf * coeff
            heatmap_avg += heatmap * coeff

        heatmav_left = xp.zeros_like(heatmap_avg)
        heatmav_left[1:, :] = heatmap_avg[:-1, :]
        heatmav_right = xp.zeros_like(heatmap_avg)
        heatmav_right[:-1, :] = heatmap_avg[1:, :]
        heatmav_up = xp.zeros_like(heatmap_avg)
        heatmav_up[:, 1:] = heatmap_avg[:, :-1]
        heatmav_down = xp.zeros_like(heatmap_avg)
        heatmav_down[:, :-1] = heatmap_avg[:, 1:]
        peaks_binary = (heatmap_avg >= heatmav_left) & \
                       (heatmap_avg >= heatmav_right) & \
                       (heatmap_avg >= heatmav_up) & \
                       (heatmap_avg >= heatmav_down) & \
                       (heatmap_avg > self.thre1)

        peaks = xp.array(xp.nonzero(peaks_binary[..., :len(self.index2limbname)-1]), dtype=np.int32).T
        peak_counter = peaks.shape[0]
        all_peaks = xp.zeros((peak_counter, 4), dtype=np.float32)
        all_peaks[:, 0] = peaks[:, 1]
        all_peaks[:, 1] = peaks[:, 0]
        all_peaks[:, 2] = heatmap_avg[peaks.T.tolist()]
        peaks_order = peaks[..., 2]
        try:
            all_peaks = all_peaks[xp.argsort(peaks_order)]
        except AttributeError:
            # cupy.argsort is not available at cupy==1.0.1
            peaks_order = chainer.cuda.to_cpu(peaks_order)
            all_peaks = all_peaks[np.argsort(peaks_order)]
        all_peaks[:, 3] = xp.arange(peak_counter, dtype=np.float32)
        if self.gpu >= 0:
            all_peaks = chainer.cuda.to_cpu(all_peaks)
            peaks_order = chainer.cuda.to_cpu(peaks_order)
        all_peaks = np.split(all_peaks, np.cumsum(
            np.bincount(peaks_order, minlength=len(self.index2limbname)-1)))
        connection_all = []
        mid_num = 10
        eps = 1e-8
        score_mid = paf_avg[:, :, [[x - 19 for x in self.map_idx[k]]
                                   for k in range(len(self.map_idx))]]

        cands = np.array(all_peaks, dtype=object)[
            np.array(self.limb_sequence, dtype=np.int32) - 1]
        candAs = cands[:, 0]
        candBs = cands[:, 1]
        nAs = np.array([len(candA) for candA in candAs])
        nBs = np.array([len(candB) for candB in candBs])
        target_indices = np.nonzero(np.logical_and(nAs != 0, nBs != 0))[0]
        if len(target_indices) == 0:
            return [], []

        all_candidates_A = [np.repeat(np.array(tmp_candA, dtype=np.float32), nB, axis=0)
                            for tmp_candA, nB in zip(candAs, nBs)]
        all_candidates_B = [np.tile(np.array(tmp_candB, dtype=np.float32), (nA, 1))
                            for tmp_candB, nA in zip(candBs, nAs)]

        target_candidates_B = [all_candidates_B[index]
                               for index in target_indices]
        target_candidates_A = [all_candidates_A[index]
                               for index in target_indices]

        vec = np.vstack(target_candidates_B)[
            :, :2] - np.vstack(target_candidates_A)[:, :2]
        if self.gpu >= 0:
            vec = chainer.cuda.to_gpu(vec)
        norm = xp.sqrt(xp.sum(vec ** 2, axis=1)) + eps
        vec = vec / norm[:, None]
        start_end = zip(np.round(np.mgrid[np.vstack(target_candidates_A)[:, 1].reshape(-1, 1):np.vstack(target_candidates_B)[:, 1].reshape(-1, 1):(mid_num * 1j)]).astype(np.int32),
                        np.round(np.mgrid[np.vstack(target_candidates_A)[:, 0].reshape(-1, 1):np.vstack(
                            target_candidates_B)[:, 0].reshape(-1, 1):(mid_num * 1j)]).astype(np.int32),
                        np.concatenate([[[index] * mid_num for i in range(len(c))] for index, c in zip(target_indices, target_candidates_B)]),)

        v = score_mid[np.concatenate(
            start_end, axis=1).tolist()].reshape(-1, mid_num, 2)
        score_midpts = xp.sum(v * xp.repeat(vec, (mid_num),
                                            axis=0).reshape(-1, mid_num, 2), axis=2)
        score_with_dist_prior = xp.sum(score_midpts, axis=1) / mid_num + \
            xp.minimum(0.5 * bgr_img.shape[0] / norm - 1,
                       xp.zeros_like(norm, dtype=np.float32))
        c1 = xp.sum(score_midpts > self.thre2, axis=1) > 0.8 * mid_num
        c2 = score_with_dist_prior > 0.0
        criterion = xp.logical_and(c1, c2)

        indices_bins = np.cumsum(nAs * nBs)
        indices_bins = np.concatenate(
            [np.zeros(1), indices_bins]).astype(np.int32)
        target_candidate_indices = xp.nonzero(criterion)[0]
        if self.gpu >= 0:
            target_candidate_indices = chainer.cuda.to_cpu(
                target_candidate_indices)
            score_with_dist_prior = chainer.cuda.to_cpu(score_with_dist_prior)

        k_s = np.digitize(target_candidate_indices, indices_bins) - 1
        i_s = (target_candidate_indices - (indices_bins[k_s])) // nBs[k_s]
        j_s = (target_candidate_indices - (indices_bins[k_s])) % nBs[k_s]

        connection_candidate = np.concatenate([k_s.reshape(-1, 1),
                                               i_s.reshape(-1, 1),
                                               j_s.reshape(-1, 1),
                                               score_with_dist_prior[
                                                   target_candidate_indices][None, ].T,
                                               (score_with_dist_prior[target_candidate_indices][None, ] +
                                                np.concatenate(target_candidates_A)[target_candidate_indices, 2] + np.concatenate(target_candidates_B)[target_candidate_indices, 2]).T], axis=1)

        sorted_indices = np.argsort(
            connection_candidate[:, 0] * 100 - connection_candidate[:, 3])

        connection_all = []
        for _ in range(0, 19):
            connection = np.zeros((0, 5), dtype=np.float32)
            connection_all.append(connection)

        for c_candidate in connection_candidate[sorted_indices]:
            k, i, j = c_candidate[0:3].astype(np.int32)
            score = c_candidate[3]
            if(len(connection_all[k]) >= min(nAs[k], nBs[k])):
                continue
            i *= nBs[k]
            if(i not in connection_all[k][:, 3] and j not in connection_all[k][:, 4]):
                connection_all[k] = np.vstack([connection_all[k], np.array(
                    [all_candidates_A[k][i][3], all_candidates_B[k][j][3], score, i, j], dtype=np.float32)])

        joint_cands_indices = -1 * np.ones((0, 20))
        candidate = np.array(
            [item for sublist in all_peaks for item in sublist])
        for k in range(len(self.map_idx)):
            partAs = connection_all[k][:, 0]
            partBs = connection_all[k][:, 1]
            indexA, indexB = np.array(self.limb_sequence[k]) - 1
            for i in range(len(connection_all[k])):  # = 1:size(temp,1)
                found = 0
                joint_cands_indices_idx = [-1, -1]
                # 1:size(joint_cands_indices,1):
                for j in range(len(joint_cands_indices)):
                    if joint_cands_indices[j][indexA] == float(partAs[i]) or joint_cands_indices[j][indexB] == float(partBs[i]):
                        joint_cands_indices_idx[found] = j
                        found += 1

                if found == 1:
                    j = joint_cands_indices_idx[0]
                    if(joint_cands_indices[j][indexB] != float(partBs[i])):
                        joint_cands_indices[j][indexB] = partBs[i]
                        joint_cands_indices[j][-1] += 1
                        joint_cands_indices[
                            j][-2] += candidate[partBs[i].astype(int), 2] + connection_all[k][i][2]
                        joint_cands_indices[
                            j][-2] += candidate[partBs[i].astype(int), 2] + connection_all[k][i][2]
                elif found == 2:  # if found 2 and disjoint, merge them
                    j1, j2 = joint_cands_indices_idx
                    membership = ((joint_cands_indices[j1] >= 0).astype(
                        int) + (joint_cands_indices[j2] >= 0).astype(int))[:-2]
                    if len(np.nonzero(membership == 2)[0]) == 0:  # merge
                        joint_cands_indices[j1][
                            :-2] += (joint_cands_indices[j2][:-2] + 1)
                        joint_cands_indices[
                            j1][-2:] += joint_cands_indices[j2][-2:]
                        joint_cands_indices[j1][-2] += connection_all[k][i][2]
                        joint_cands_indices = np.delete(
                            joint_cands_indices, j2, 0)
                    else:  # as like found == 1
                        joint_cands_indices[j1][indexB] = partBs[i]
                        joint_cands_indices[j1][-1] += 1
                        joint_cands_indices[
                            j1][-2] += candidate[partBs[i].astype(int), 2] + connection_all[k][i][2]

                # if find no partA in the joint_cands_indices, create a new
                # joint_cands_indices
                elif not found and k < len(self.index2limbname) - 2:
                    row = -1 * np.ones(20)
                    row[indexA] = partAs[i]
                    row[indexB] = partBs[i]
                    row[-1] = 2
                    row[-2] = sum(candidate[connection_all[k]
                                            [i, :2].astype(int), 2]) + connection_all[k][i][2]
                    joint_cands_indices = np.vstack([joint_cands_indices, row])

        # delete some rows of joint_cands_indices which has few parts occur
        deleteIdx = []
        for i in range(len(joint_cands_indices)):
            if joint_cands_indices[i][-1] < 4 or joint_cands_indices[i][-2] / joint_cands_indices[i][-1] < 0.4:
                deleteIdx.append(i)
        joint_cands_indices = np.delete(joint_cands_indices, deleteIdx, axis=0)

        return self._extract_joint_position(joint_cands_indices, candidate), all_peaks

    def _extract_joint_position(self, joint_cands_indices, candidate):
        people_joint_positions = []
        for joint_cand_idx in joint_cands_indices:
            person_joint_positions = []
            for i, limb_name in enumerate(self.index2limbname):
                cand_idx = int(joint_cand_idx[i])
                if cand_idx == -1 or cand_idx >= candidate.shape[0]:
                    person_joint_positions.append(dict(limb=limb_name,
                                                       x=0,
                                                       y=0,
                                                       score=-1))
                    continue
                X, Y = candidate[cand_idx, :2]
                person_joint_positions.append(dict(limb=limb_name,
                                                   x=X,
                                                   y=Y,
                                                   score=candidate[cand_idx, 2]))
            people_joint_positions.append(person_joint_positions)

        return people_joint_positions

    def _draw_joints(self, img, people_joint_positions, all_peaks):
        if all_peaks:
            # keypoints
            cmap = matplotlib.cm.get_cmap('hsv')
            n = len(self.index2limbname)-1
            for i in range(len(self.index2limbname)-1):
                rgba = np.array(cmap(1. * i / n))
                color = rgba[:3] * 255
                for j in range(len(all_peaks[i])):
                    cv2.circle(img, (int(all_peaks[i][j][0]), int(
                        all_peaks[i][j][1])), 4, color, thickness=-1)

        # connections
        stickwidth = 4
        for joint_positions in people_joint_positions:
            n = len(self.limb_sequence)
            for i, conn in enumerate(self.limb_sequence):
                rgba = np.array(cmap(1. * i / n))
                color = rgba[:3] * 255
                j1, j2 = joint_positions[conn[0]-1], joint_positions[conn[1]-1]
                if j1['score'] < 0 or j2['score'] < 0:
                    continue
                cx, cy = int((j1['x'] + j2['x']) / 2.), int((j1['y'] + j2['y']) / 2.)
                dx, dy = j1['x'] - j2['x'], j1['y'] - j2['y']
                length = np.linalg.norm([dx, dy])
                angle = int(np.degrees(np.arctan2(dy, dx)))
                polygon = cv2.ellipse2Poly((cx, cy), (int(length / 2.), stickwidth),
                                           angle, 0, 360, 1)
                top, left = np.min(polygon[:,1]), np.min(polygon[:,0])
                bottom, right = np.max(polygon[:,1]), np.max(polygon[:,0])
                roi = img[top:bottom,left:right]
                roi2 = roi.copy()
                cv2.fillConvexPoly(roi2, polygon - np.array([left, top]), color)
                cv2.addWeighted(roi, 0.4, roi2, 0.6, 0.0, dst=roi)

        # for hand
        if self.use_hand:
            offset = len(self.limb_sequence)
            for joint_positions in people_joint_positions:
                n = len(joint_positions[offset:])
                for i, jt in enumerate(joint_positions[offset:]):
                    if jt['score'] < 0.0:
                        continue
                    rgba = np.array(cmap(1. * i / n))
                    color = rgba[:3] * 255
                    cv2.circle(img, (int(jt['x']), int(jt['y'])),
                               2, color, thickness=-1)

            for joint_positions in people_joint_positions:
                offset = len(self.limb_sequence)
                n = len(self.hand_sequence)
                for _ in range(2):
                    # for both hands
                    for i, conn in enumerate(self.hand_sequence):
                        rgba = np.array(cmap(1. * i / n))
                        color = rgba[:3] * 255
                        j1 = joint_positions[offset + conn[0]]
                        j2 = joint_positions[offset + conn[1]]
                        if j1['score'] < 0 or j2['score'] < 0:
                            continue
                        cx, cy = int((j1['x'] + j2['x']) / 2.), int((j1['y'] + j2['y']) / 2.)
                        dx, dy = j1['x'] - j2['x'], j1['y'] - j2['y']
                        length = np.linalg.norm([dx, dy])
                        angle = int(np.degrees(np.arctan2(dy, dx)))
                        polygon = cv2.ellipse2Poly((cx, cy), (int(length / 2.), stickwidth),
                                                   angle, 0, 360, 1)
                        top, left = np.min(polygon[:,1]), np.min(polygon[:,0])
                        bottom, right = np.max(polygon[:,1]), np.max(polygon[:,0])
                        roi = img[top:bottom,left:right]
                        roi2 = roi.copy()
                        cv2.fillConvexPoly(roi2, polygon - np.array([left, top]), color)
                        cv2.addWeighted(roi, 0.4, roi2, 0.6, 0.0, dst=roi)
                    #
                    offset += len(self.index2handname) / 2

        return img

    def hand_estimate(self, bgr, people_joint_positions):
        if self.backend == 'chainer':
            return self._hand_estimate_chainer_backend(bgr, people_joint_positions)
        raise ValueError('Unsupported backend: {0}'.format(self.backend))

    def _hand_estimate_chainer_backend(self, bgr, people_joint_positions):
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        # https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/29ea7e24dce4abae30faecf769855823ad7bb637/src/openpose/hand/handDetector.cpp
        for joint_positions in people_joint_positions:
            # crop hand image for each person
            width = self._get_hand_roi_width(joint_positions) + self.hand_width_offset
            hand_joint_positions = []
            if width > self.hand_thre1:
                rwrist = find_joint('RWrist', joint_positions)
                if rwrist['score'] > self.hand_thre2:
                    cx, cy = rwrist['x'], rwrist['y']
                    relbow = find_joint('RElbow', joint_positions)
                    if relbow['score'] > 0:
                        cx += 0.3 * (cx - relbow['x'])
                        cy += 0.3 * (cy - relbow['y'])
                    hand_bgr = self._crop_square_image(bgr, cx, cy, width)
                    hand_joints = self._hand_estimate_chainer_backend_each(
                        hand_bgr, cx, cy, False)
                    hand_joint_positions.extend(hand_joints)
                    if self.visualize:
                        cv2.circle(bgr, (int(cx), int(cy)), int(width/2.),
                                   (255, 0, 0), thickness=1)

                lwrist = find_joint('LWrist', joint_positions)
                if lwrist['score'] > self.hand_thre2:
                    cx, cy = lwrist['x'], lwrist['y']
                    lelbow = find_joint('LElbow', joint_positions)
                    if lelbow['score'] > 0:
                        cx += 0.3 * (cx - lelbow['x'])
                        cy += 0.3 * (cy - lelbow['y'])
                    hand_bgr = self._crop_square_image(bgr, cx, cy, width)
                    hand_joints = self._hand_estimate_chainer_backend_each(
                        hand_bgr, cx, cy, True)
                    hand_joint_positions.extend(hand_joints)
                    if self.visualize:
                        cv2.circle(bgr, (int(cx), int(cy)), int(width/2.),
                                   (255, 0, 0), thickness=1)

            for limb in self.index2handname:
                jt = find_joint(limb, hand_joint_positions)
                if jt is not None:
                    joint_positions.append(jt)
                else:
                    joint_positions.append({
                        'x': 0, 'y': 0, 'score': -1, 'limb': limb})

        return people_joint_positions

    def _hand_estimate_chainer_backend_each(self, hand_bgr, cx, cy, left_hand):
        xp = self.hand_net.xp

        if left_hand:
            hand_bgr = cv2.flip(hand_bgr, 1)  # 1 = vertical

        resized = cv2.resize(hand_bgr, (368, 368), interpolation=cv2.INTER_CUBIC)
        x = np.array(resized[np.newaxis], dtype=np.float32)
        x = x.transpose(0, 3, 1, 2)
        x = x / 256 - 0.5

        if self.gpu >= 0:
            x = chainer.cuda.to_gpu(x)
        x = chainer.Variable(x)

        heatmaps = self.hand_net(x)
        heatmaps = F.resize_images(heatmaps[-1], hand_bgr.shape[:2])[0]
        if self.gpu >= 0:
            heatmaps.to_cpu()
        heatmaps = heatmaps.array

        if left_hand:
            heatmaps = heatmaps.transpose(1, 2, 0)
            heatmaps = cv2.flip(heatmaps, 1)
            heatmaps = heatmaps.transpose(2, 0, 1)

        # get peak on heatmap
        hmaps = []
        if xp == np:
            # cpu
            for i in range(heatmaps.shape[0] - 1):
                heatmap = gaussian_filter(heatmaps[i], sigma=self.hand_gaussian_sigma)
                hmaps.append(heatmap)
        else:
            heatmaps = chainer.cuda.to_gpu(heatmaps)
            heatmaps = F.convolution_2d(
                heatmaps[:, xp.newaxis], self.hand_gaussian_kernel,
                stride=1, pad=int(self.hand_gaussian_ksize / 2))
            heatmaps = chainer.cuda.to_cpu(xp.squeeze(heatmaps.array))
            for heatmap in heatmaps[:-1]:
                hmaps.append(heatmap)
        keypoints = []
        idx_offset = 0
        if left_hand:
            idx_offset += len(hmaps)
        for i, heatmap in enumerate(hmaps):
            conf = heatmap.max()
            cds = np.array(np.where(heatmap==conf)).flatten().tolist()
            py = cy + cds[0] - hand_bgr.shape[0] / 2
            px = cx + cds[1] - hand_bgr.shape[1] / 2
            keypoints.append({'x': px, 'y': py, 'score': conf,
                              'limb': self.index2handname[idx_offset+i]})
        return keypoints

    def _crop_square_image(self, img, cx, cy, width):
        cx, cy, width = int(cx), int(cy), int(width)
        left, right = cx - int(width / 2), cx + int(width / 2)
        top, bottom = cy - int(width / 2), cy + int(width / 2)
        imh, imw, imc = img.shape
        cropped = img[max(0, top):max(min(imh, bottom), 0), max(0, left):max(min(imw, right), 0)]
        ch, cw = cropped.shape[:2]
        bx, by = max(0, -left), max(0, -top)
        padded = np.zeros((bottom - top, right - left, imc), dtype=np.uint8)
        padded[by:by+ch,bx:bx+cw] = cropped
        return padded

    def _get_hand_roi_width(self, joint_positions):
        lengths = []
        for conn, ratio in zip(self.limb_sequence, self.limb_length_hand_ratio):
            j1, j2 = joint_positions[conn[0]-1], joint_positions[conn[1]-1]
            length = 0
            if j1['score'] > 0 and j2['score'] > 0:
                dx, dy = j1['x'] - j2['x'], j1['y'] - j2['y']
                length = np.linalg.norm([dx, dy])
            lengths.append(length / ratio)
        if np.sum(lengths[:5]) > 0:
            lengths = lengths[:5]
        rospy.logdebug('length: %s' % lengths)
        return np.sum(lengths) / len(np.nonzero(lengths)[0])


if __name__ == '__main__':
    rospy.init_node('people_pose_estimation_2d')
    PeoplePoseEstimation2D()
    rospy.spin()
