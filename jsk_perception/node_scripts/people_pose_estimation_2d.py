#!/usr/bin/env python
# -*- coding:utf-8 -*-

#    See: https://arxiv.org/abs/1611.08050

import math

import chainer
import chainer.functions as F
from chainer import cuda
import cv2
import matplotlib
import numpy as np
import pylab as plt  # NOQA

import cv_bridge
import message_filters
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from openpose.net import OpenPoseNet


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
    # find connection in the specified sequence,
    # center 29 is in the position 15
    limb_sequence = [[2, 3], [2, 6], [3, 4], [4, 5], [6, 7], [7, 8], [2, 9],
                     [9, 10], [10, 11], [2, 12], [12, 13], [13, 14], [2, 1],
                     [1, 15], [15, 17], [1, 16], [16, 18], [3, 17], [6, 18]]
    # the middle joints heatmap correpondence
    map_idx = [[31, 32], [39, 40], [33, 34], [35, 36], [41, 42], [43, 44],
               [19, 20], [21, 22], [23, 24], [25, 26], [27, 28], [29, 30],
               [47, 48], [49, 50], [53, 54], [51, 52], [55, 56], [37, 38],
               [45, 46]]

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

    colors = [[255, 0, 0], [255, 85, 0], [255, 170, 0], [255, 255, 0], [170, 255, 0], [85, 255, 0], [0, 255, 0],
              [0, 255, 85], [0, 255, 170], [0, 255, 255], [
                  0, 170, 255], [0, 85, 255], [0, 0, 255], [85, 0, 255],
              [170, 0, 255], [255, 0, 255], [255, 0, 170], [255, 0, 85]]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.backend = rospy.get_param('~backend', 'chainer')
        self.scales = rospy.get_param('~scales', [0.38])
        self.stride = rospy.get_param('~stride', 8)
        self.pad_value = rospy.get_param('~pad_value', 128)
        self.thre1 = rospy.get_param('~thre1', 0.1)
        self.thre2 = rospy.get_param('~thre2', 0.05)
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        self.with_depth = rospy.get_param('~with_depth', False)
        self._load_model()
        self.image_pub = self.advertise('~output', Image, queue_size=1)
        self.pose_pub = self.advertise('~pose', PeoplePoseArray, queue_size=1)
        self.sub_info = None
        if self.with_depth is True:
            self.pose_2d_pub = self.advertise('~pose_2d', PeoplePoseArray, queue_size=1)

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
        self.net = OpenPoseNet()
        chainer.serializers.load_npz(model_file, self.net)
        rospy.loginfo('Finished loading trained model: {0}'.format(model_file))
        if self.gpu != -1:
            self.net.to_gpu(self.gpu)

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

        pose_estimated_img, people_joint_positions = self.pose_estimate(img)
        pose_estimated_msg = br.cv2_to_imgmsg(
            pose_estimated_img.astype(np.uint8), encoding='bgr8')
        pose_estimated_msg.header = img_msg.header

        people_pose_msg = PeoplePoseArray()
        people_pose_msg.header = img_msg.header
        people_pose_2d_msg = self._create_2d_people_pose_array_msgs(
            people_joint_positions,
            img_msg.header)

        # calculate xyz-position
        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]
        for person_joint_positions in people_joint_positions:
            pose_msg = PeoplePose()
            for joint_pos in person_joint_positions:
                if joint_pos['score'] < 0:
                    continue
                z = float(depth_img[int(joint_pos['y'])][int(joint_pos['x'])])
                if np.isnan(z):
                    continue
                x = (joint_pos['x'] - cx) * z / fx
                y = (joint_pos['y'] - cy) * z / fy
                pose_msg.limb_names.append(joint_pos['limb'])
                pose_msg.scores.append(joint_pos['score'])
                pose_msg.poses.append(Pose(position=Point(x=x, y=y, z=z),
                                           orientation=Quaternion(w=1)))
            people_pose_msg.poses.append(pose_msg)

        self.pose_2d_pub.publish(people_pose_2d_msg)
        self.pose_pub.publish(people_pose_msg)
        if self.visualize:
            self.image_pub.publish(pose_estimated_msg)

    def _cb(self, img_msg):
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        pose_estimated_img, people_joint_positions = self.pose_estimate(img)
        pose_estimated_msg = br.cv2_to_imgmsg(
            pose_estimated_img.astype(np.uint8), encoding='bgr8')
        pose_estimated_msg.header = img_msg.header

        people_pose_msg = self._create_2d_people_pose_array_msgs(
            people_joint_positions,
            img_msg.header)

        self.pose_pub.publish(people_pose_msg)
        if self.visualize:
            self.image_pub.publish(pose_estimated_msg)

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
        xp = cuda.cupy if self.gpu != -1 else np

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
            if self.gpu != -1:
                x = chainer.cuda.to_gpu(x)
            x = chainer.Variable(x)
            pafs, heatmaps = self.net(x)
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
        if self.gpu != -1:
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
            return bgr_img, []

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
        if self.gpu != -1:
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
        if self.gpu != -1:
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

        if self.visualize:
            result_img = self._visualize(
                bgr_img, joint_cands_indices, all_peaks, candidate)
        else:
            result_img = bgr_img

        return result_img, self._extract_joint_position(joint_cands_indices, candidate)

    def _visualize(self, img, joint_cands_indices, all_peaks, candidate):

        cmap = matplotlib.cm.get_cmap('hsv')
        for i in range(len(self.index2limbname)-1):
            rgba = np.array(cmap(1 - i / 18. - 1. / 36))
            rgba[0:3] *= 255
            for j in range(len(all_peaks[i])):
                cv2.circle(img, (int(all_peaks[i][j][0]), int(
                    all_peaks[i][j][1])), 4, self.colors[i], thickness=-1)

        stickwidth = 4
        for i in range(len(self.index2limbname) - 2):
            for joint_cand_indices in joint_cands_indices:
                index = joint_cand_indices[np.array(self.limb_sequence[i],
                                                    dtype=np.int32) - 1]
                if -1 in index:
                    continue
                cur_img = img.copy()
                Y = candidate[index.astype(int), 0]
                X = candidate[index.astype(int), 1]
                mX = np.mean(X)
                mY = np.mean(Y)
                length = ((X[0] - X[1]) ** 2 + (Y[0] - Y[1]) ** 2) ** 0.5
                angle = math.degrees(math.atan2(X[0] - X[1], Y[0] - Y[1]))
                polygon = cv2.ellipse2Poly((int(mY), int(mX)), (int(
                    length / 2), stickwidth), int(angle), 0, 360, 1)
                cv2.fillConvexPoly(cur_img, polygon, self.colors[i])
                img = cv2.addWeighted(img, 0.4, cur_img, 0.6, 0)

        return img

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


if __name__ == '__main__':
    rospy.init_node('people_pose_estimation_2d')
    PeoplePoseEstimation2D()
    rospy.spin()
