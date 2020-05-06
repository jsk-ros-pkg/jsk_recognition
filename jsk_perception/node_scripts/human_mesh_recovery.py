#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Ros Wrapper of Human Mesh Recovery
#    See: End-to-end Recovery of Human Shape and Pose
#         https://akanazawa.github.io/hmr/

from __future__ import print_function

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
from chainer import Variable
import cv2
import numpy as np
import pylab as plt  # NOQA

import tf
import cv_bridge
import message_filters
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import PeoplePose
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import Image

from hmr.smpl import SMPL
from hmr.net import EncoderFC3Dropout
from hmr.resnet_v2_50 import ResNet_v2_50


def format_pose_msg(person_pose):
    key_points = []
    for pose, score in zip(person_pose.poses, person_pose.scores):
        key_points.append(pose.position.x)
        key_points.append(pose.position.y)
        key_points.append(score)
    return np.array(key_points, 'f').reshape(-1, 3)


def resize_img(img, scale_factor):
    new_size = (np.floor(np.array(img.shape[0:2]) * scale_factor)).astype(int)
    new_img = cv2.resize(img, (new_size[1], new_size[0]))
    # This is scale factor of [height, width] i.e. [y, x]
    actual_factor = [
        new_size[0] / float(img.shape[0]), new_size[1] / float(img.shape[1])
    ]
    return new_img, actual_factor


def scale_and_crop(image, scale, center, img_size):
    image_scaled, scale_factors = resize_img(image, scale)
    # Swap so it's [x, y]
    scale_factors = [scale_factors[1], scale_factors[0]]
    center_scaled = np.round(center * scale_factors).astype(np.int)

    margin = int(img_size / 2)
    image_pad = np.pad(
        image_scaled, ((margin, ), (margin, ), (0, )), mode='edge')
    center_pad = center_scaled + margin
    # figure out starting point
    start_pt = center_pad - margin
    end_pt = center_pad + margin
    # crop:
    crop = image_pad[start_pt[1]:end_pt[1], start_pt[0]:end_pt[0], :]
    proc_param = {
        'scale': scale,
        'start_pt': start_pt,
        'end_pt': end_pt,
        'img_size': img_size
    }

    return crop, proc_param


def get_bbox(key_points, vis_thr=0.2):
    # Pick the most confident detection.
    vis = key_points[:, 2] > vis_thr
    vis_kp = key_points[vis, :2]
    if len(vis_kp) == 0:
        return False, False
    min_pt = np.min(vis_kp, axis=0)
    max_pt = np.max(vis_kp, axis=0)
    person_height = np.linalg.norm(max_pt - min_pt)
    if person_height == 0:
        return False, False
    center = (min_pt + max_pt) / 2.
    scale = 150. / person_height
    return scale, center


def preprocess_image(img, key_points=None, img_size=224):
    if key_points is None:
        scale = 1.
        center = np.round(np.array(img.shape[:2]) / 2).astype(int)
        # image center in (x,y)
        center = center[::-1]
    else:
        scale, center = get_bbox(key_points, vis_thr=0.1)
        if scale is False:
            scale = 1.
            center = np.round(np.array(img.shape[:2]) / 2).astype(int)
            # image center in (x,y)
            center = center[::-1]
    crop_img, proc_param = scale_and_crop(img, scale, center,
                                          img_size)

    # Normalize image to [-1, 1]
    crop_img = 2 * ((crop_img / 255.) - 0.5)
    return crop_img, proc_param


mean = np.array([[
    0.90365213, -0.00383353,  0.03301106,  3.14986515, -0.01883755,
    0.16895422, -0.15615709, -0.0058559,  0.07191881, -0.18924442,
    -0.04396844, -0.05114707,  0.24385466,  0.00881136,  0.02384637,
    0.2066803, -0.10190887, -0.03373535,  0.27340922,  0.00637481,
    0.07408072, -0.03409823, -0.00971786,  0.03841642,  0.0191336,
    0.10812955, -0.06782207, -0.08026548, -0.18373352,  0.16556455,
    0.03735897, -0.02497507,  0.02688527, -0.18802814,  0.17772846,
    0.13135587,  0.01438429,  0.15891947, -0.2279436, -0.07497088,
    0.05169746,  0.08784129,  0.02147929,  0.02320284, -0.42375749,
    -0.04963749,  0.08627309,  0.47963148,  0.26528436, -0.1028522,
    -0.02501041,  0.05762934, -0.26270828, -0.8297376,  0.13903582,
    0.30412629,  0.79824799,  0.12842464, -0.64139324,  0.16469972,
    -0.08669609,  0.55955994, -0.16742738, -0.03153928, -0.02245264,
    -0.02357809,  0.02061746,  0.02320515,  0.00869796, -0.1655257,
    -0.07094092, -0.1663706, -0.10953037,  0.11675739,  0.20495811,
    0.10592803,  0.14583197, -0.31755996,  0.13645983,  0.28833047,
    0.06303538,  0.48629287,  0.23359743, -0.02812484,  0.23948504]], 'f')


class HumanMeshRecovery(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        self.with_people_pose = rospy.get_param('~with_people_pose', False)
        self.num_stage = rospy.get_param('~num_stage', 3)

        self.smpl = SMPL()
        self.encoder_fc3_model = EncoderFC3Dropout()
        self.resnet_v2 = ResNet_v2_50()
        self._load_model()

        self.br = cv_bridge.CvBridge()
        self.pose_pub = self.advertise(
            '~output/pose', PeoplePoseArray, queue_size=1)

    def _load_model(self):
        smpl_model_file = rospy.get_param('~smpl_model_file')
        chainer.serializers.load_npz(smpl_model_file, self.smpl)
        encoder_fc3_model_file = rospy.get_param('~encoder_model_file')
        chainer.serializers.load_npz(
            encoder_fc3_model_file, self.encoder_fc3_model)
        resnet_v2_50_model_file = rospy.get_param('~resnet_v2_50_model_file')
        chainer.serializers.load_npz(resnet_v2_50_model_file, self.resnet_v2)

        rospy.loginfo('Finished loading trained model')
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
            self.smpl.to_gpu()
            self.encoder_fc3_model.to_gpu()
            self.resnet_v2.to_gpu()
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False

    def subscribe(self):
        if self.with_people_pose:
            queue_size = rospy.get_param('~queue_size', 10)
            sub_img = message_filters.Subscriber(
                '~input', Image, queue_size=queue_size, buff_size=2**24)
            sub_pose = message_filters.Subscriber(
                '~input/pose', PeoplePoseArray,
                queue_size=queue_size, buff_size=2**24)
            self.subs = [sub_img, sub_pose]

            if rospy.get_param('~approximate_sync', False):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    fs=self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self._cb_with_pose)
        else:
            sub_img = rospy.Subscriber(
                '~input', Image, self._cb,
                queue_size=1, buff_size=2**24)
            self.subs = [sub_img]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb(self, img_msg):
        br = self.br
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        img, _ = preprocess_image(img)
        imgs = img.transpose(2, 0, 1)[None, ]
        verts, Js, Rs, A, cams, poses, shapes = self.pose_estimate(imgs)

        people_pose_msg = self._create_people_pose_array_msgs(
            chainer.cuda.to_cpu(A.data), img_msg.header)
        self.pose_pub.publish(people_pose_msg)

    def _cb_with_pose(self, img_msg, people_pose_msg):
        br = self.br
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        imgs = []
        for person_pose in people_pose_msg.poses:
            key_points = format_pose_msg(person_pose)
            crop_img, _ = preprocess_image(img, key_points)
            imgs.append(crop_img)
        if len(imgs) == 0:
            img, _ = preprocess_image(img)
            imgs = np.array(img[None, ], 'f').transpose(0, 3, 1, 2)
        else:
            imgs = np.array(imgs, 'f').transpose(0, 3, 1, 2)
        verts, Js, Rs, A, cams, poses, shapes = self.pose_estimate(imgs)

        people_pose_msg = self._create_people_pose_array_msgs(
            chainer.cuda.to_cpu(A.data), img_msg.header)
        self.pose_pub.publish(people_pose_msg)

    def _create_people_pose_array_msgs(self, people_joint_positions, header):
        people_pose_msg = PeoplePoseArray(header=header)
        for i, person_joint_positions in enumerate(people_joint_positions):
            pose_msg = PeoplePose()
            for joint_pose in person_joint_positions:
                pose_msg.limb_names.append(str(i))
                pose_msg.scores.append(0.0)
                q_xyzw = tf.transformations.quaternion_from_matrix(joint_pose)
                pose_msg.poses.append(
                    Pose(position=Point(
                        x=joint_pose[0, 3],
                        y=joint_pose[1, 3],
                        z=joint_pose[2, 3]),
                        orientation=Quaternion(
                        x=q_xyzw[0],
                        y=q_xyzw[1],
                        z=q_xyzw[2],
                        w=q_xyzw[3])))
            people_pose_msg.poses.append(pose_msg)
        return people_pose_msg

    def pose_estimate(self, imgs):
        batch_size = imgs.shape[0]
        imgs = Variable(self.resnet_v2.xp.array(imgs, 'f'))
        img_feat = self.resnet_v2(imgs).reshape(batch_size, -1)

        theta_prev = F.tile(
            Variable(self.encoder_fc3_model.xp.array(mean, 'f')),
            (batch_size, 1))
        num_cam = 3
        num_theta = 72
        for i in range(self.num_stage):
            state = F.concat([img_feat, theta_prev], axis=1)
            delta_theta = self.encoder_fc3_model(state)
            theta_here = theta_prev + delta_theta
            # cam = N x 3, pose N x self.num_theta, shape: N x 10
            cams = theta_here[:, :num_cam]
            poses = theta_here[:, num_cam:(num_cam + num_theta)]
            shapes = theta_here[:, (num_cam + num_theta):]

            verts, Js, Rs, A = self.smpl(shapes, poses)
            # Project to 2D!
            # pred_kp = batch_orth_proj_idrot(
            #     Js, cams, name='proj_2d_stage%d' % i)
            theta_prev = theta_here
        return verts, Js, Rs, A, cams, poses, shapes


if __name__ == '__main__':
    rospy.init_node('human_mesh_recovery')
    HumanMeshRecovery()
    rospy.spin()
