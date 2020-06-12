#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function

import cv2
import numpy as np
import os
import traceback

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
from chainer.dataset import download
from chainer.serializers import load_npz
import chainer.functions as F
import chainer.links as L

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
import tf.transformations as T

from geometry_msgs.msg import PoseArray
from jsk_perception.cfg import FacePoseEstimationConfig as Config
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import PeoplePoseArray
from sensor_msgs.msg import Image


def _get(var):
    if isinstance(var, chainer.Variable):
        var = var.data
        if hasattr(var, 'get'):
            var = var.get()
    return var


def _project_plane_yz(vec):
    x = vec.dot(np.array([0, 1, 0], dtype=np.float32))
    y = vec.dot(np.array([0, 0, 1], dtype=np.float32))
    return np.array([x, -y], dtype=np.float32)  # y flip


def _draw_line(img, pt1, pt2, color, thickness=2):
    pt1 = (int(pt1[0]), int(pt1[1]))
    pt2 = (int(pt2[0]), int(pt2[1]))
    return cv2.line(img, pt1, pt2, color, int(thickness))


class HyperFaceModel(chainer.Chain):

    def __init__(self, pretrained_model='auto'):
        super(HyperFaceModel, self).__init__(
            conv1  = L.Convolution2D(3,   96, 11, stride=4, pad=0),
            conv1a = L.Convolution2D(96,  256, 4, stride=4, pad=0),
            conv2  = L.Convolution2D(96,  256, 5, stride=1, pad=2),
            conv3  = L.Convolution2D(256, 384, 3, stride=1, pad=1),
            conv3a = L.Convolution2D(384, 256, 2, stride=2, pad=0),
            conv4  = L.Convolution2D(384, 384, 3, stride=1, pad=1),
            conv5  = L.Convolution2D(384, 256, 3, stride=1, pad=1),
            conv_all = L.Convolution2D(768, 192, 1, stride=1, pad=0),
            fc_full  = L.Linear(6 * 6 * 192, 3072),
            fc_detection1  = L.Linear(3072, 512),
            fc_detection2  = L.Linear(512,  2),
            fc_landmarks1  = L.Linear(3072, 512),
            fc_landmarks2  = L.Linear(512,  42),
            fc_visibility1 = L.Linear(3072, 512),
            fc_visibility2 = L.Linear(512,  21),
            fc_pose1       = L.Linear(3072, 512),
            fc_pose2       = L.Linear(512,  3),
            fc_gender1     = L.Linear(3072, 512),
            fc_gender2     = L.Linear(512,  2),
        )

        # download pretrained weights
        if pretrained_model == 'auto':
            rospy.loginfo("Loading pretrained model. (This may take some minutes.)")
            url = 'https://jsk-ros-pkg.s3.amazonaws.com/chainer/hyperface_model_epoch_190.npz'
            load_npz(download.cached_download(url), self)
            rospy.loginfo("Model loaded")
        elif pretrained_model:
            rospy.loginfo("Loading pretrained model: %s" % pretrained_model)
            load_npz(pretrained_model, self)
            rospy.loginfo("Model loaded")
        else:
            rospy.logwarn("No pretrained model is loaded.")

    def __call__(self, x):
        c1 = F.relu(self.conv1(x))
        m1 = F.max_pooling_2d(c1, 3, stride=2, pad=0)
        m1_n = F.local_response_normalization(m1)
        c1a = F.relu(self.conv1a(m1_n))
        c2 = F.relu(self.conv2(m1_n))
        m2 = F.max_pooling_2d(c2, 3, stride=2, pad=0)
        m2_n = F.local_response_normalization(m2)
        c3 = F.relu(self.conv3(m2_n))
        c3a = F.relu(self.conv3a(c3))
        c4 = F.relu(self.conv4(c3))
        c5 = F.relu(self.conv5(c4))
        m5 = F.max_pooling_2d(c5, 3, stride=2, pad=0)

        c = F.concat((c1a, c3a, m5))

        c_all = F.relu(self.conv_all(c))
        fc = F.relu(self.fc_full(c_all))

        detection = F.relu(self.fc_detection1(fc))
        detection = self.fc_detection2(detection)
        detection = F.softmax(detection)
        landmark = F.relu(self.fc_landmarks1(fc))
        landmark = self.fc_landmarks2(landmark)
        visibility = F.relu(self.fc_visibility1(fc))
        visibility = self.fc_visibility2(visibility)
        pose = F.relu(self.fc_pose1(fc))
        pose = self.fc_pose2(pose)
        gender = F.relu(self.fc_gender1(fc))
        gender = self.fc_gender2(gender)
        gender = F.softmax(gender)

        detection = F.softmax(detection)[:, 1]
        gender = F.softmax(gender)[:, 1]

        return {'detection': detection,
                'landmark': landmark,
                'visibility': visibility,
                'gender': gender,
                'pose': pose}


class HyperFacePredictor(object):
    def __init__(self, model, gpu=-1):
        assert isinstance(model, HyperFaceModel)
        self.gpu = gpu

        model.train = False
        model.report = False
        model.backward = False

        if self.gpu >= 0:
            model.to_gpu(self.gpu)

        self.model = model

    def preprocess(self, img):
        # assertion
        assert img.size > 0
        orig_h, orig_w, _ = img.shape
        assert orig_h > 0 and orig_w > 0

        # transform image
        img = img.astype(np.float32) / 255.0
        img = cv2.resize(img, (227, 227))
        img = cv2.normalize(img, None, -0.5, 0.5, cv2.NORM_MINMAX)
        img = np.transpose(img, (2, 0, 1))  # CHW

        return img

    def forward(self, imgs):
        xp = self.model.xp
        imgs = xp.asarray([self.preprocess(img) for img in imgs])

        # forwarding
        x = chainer.Variable(imgs)
        if self.gpu >= 0:
            x.to_gpu()
        y = self.model(x)

        detection = _get(y["detection"])
        landmark = _get(y["landmark"])
        visibility = _get(y["visibility"])
        pose = _get(y["pose"])
        gender = _get(y["gender"])

        result = []
        for i in range(len(detection)):
            result.append({
                "detection": detection[i],
                "landmark": landmark[i],
                "visibility": visibility[i],
                "pose": pose[i],
                "gender": gender[i]
            })
        return result

    def __call__(self, imgs):
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        return self.forward(imgs)


class FacePoseEstimator(ConnectionBasedTransport):
    def __init__(self):
        super(FacePoseEstimator, self).__init__()

        self.cv_bridge = cv_bridge.CvBridge()

        gpu = rospy.get_param("~gpu", -1)  # -1 == cpu only

        model = HyperFaceModel(pretrained_model=rospy.get_param("~model_path", None))
        self.predictor = HyperFacePredictor(model=model, gpu=gpu)
        rospy.loginfo("hyperface predictor initialized ({})".format(
            "GPU: %d" % gpu if gpu >= 0 else "CPU mode"))

        self.approximate_sync = False
        self.queue_size = 10
        self.slop = 0.1
        self.classifier_name = rospy.get_param("~classifier_name", rospy.get_name())
        self.srv = Server(Config, self.config_callback)

        self.pub_pose = self.advertise("~output/pose", PoseArray, queue_size=1)
        self.pub_gender = self.advertise("~output/gender", ClassificationResult, queue_size=1)
        self.pub_rect = self.advertise("~output/rects", RectArray, queue_size=1)
        self.pub_debug_image = self.advertise("~output/debug", Image, queue_size=1)

    @property
    def visualize(self):
        return self.pub_debug_image.get_num_connections() > 0

    def config_callback(self, config, level):
        need_resubscribe = (
            self.approximate_sync != config.approximate_sync or
            self.queue_size != config.queue_size or
            self.slop != self.slop)
        self.approximate_sync = config.approximate_sync
        self.queue_size = config.queue_size
        self.slop = config.slop

        self.face_padding = config.face_padding
        self.face_threshold = config.face_threshold

        if need_resubscribe and self.is_subscribed():
            self.unsubscribe()
            self.subscribe()

        return config

    def subscribe(self):
        sub_image = message_filters.Subscriber("~input", Image)
        self.subscribers = [
            message_filters.Subscriber("~input", Image),
            message_filters.Subscriber("~input/pose_2d", PeoplePoseArray),
            message_filters.Subscriber("~input/pose", PeoplePoseArray),
        ]

        if self.approximate_sync:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                self.subscribers, self.queue_size, self.slop)
        else:
            self.sync = message_filters.TimeSynchronizer(
                self.subscribers, self.queue_size)
        self.sync.registerCallback(self.callback)

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()

    def callback(self, img, pose2d, pose3d):
        header = img.header
        try:
            img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("Failed to convert image: %s" % str(e))
            rospy.logerr(traceback.format_exc())
            return

        faces = []
        rects = []
        face_origins = []
        face_2d_origins = []
        for p2d, p3d in zip(pose2d.poses, pose3d.poses):
            try:
                score = p2d.scores[p2d.limb_names.index("Nose")]
                if score < self.face_threshold:
                    continue
                nose = p2d.poses[p2d.limb_names.index("Nose")]
                neck = p2d.poses[p2d.limb_names.index("Neck")]
                width_half = np.sqrt((neck.position.x - nose.position.x) ** 2 +
                                     (neck.position.y - nose.position.y) ** 2)
                width_half *= 1.0 + self.face_padding
                rect = Rect(x=int(nose.position.x-width_half),
                            y=int(nose.position.y-width_half),
                            width=int(width_half*2),
                            height=int(width_half*2))
                face_origin = p3d.poses[p3d.limb_names.index("Nose")]
                face_2d_origin = nose
            except ValueError:
                continue

            try:
                face = img[rect.y:rect.y+rect.height, rect.x:rect.x+rect.width]
                if face.size <= 0:
                    continue
            except Exception as e:
                rospy.logerr("Failed to crop image: %s" % str(e))
                rospy.logerr(traceback.format_exc())
                continue
            rects.append(rect)
            face_origins.append(face_origin)
            face_2d_origins.append(face_2d_origin)
            faces.append(face)

        if not faces:
            rospy.logdebug("No face found")
            return

        try:
            results = self.predictor(faces)
        except OverflowError:
            rospy.logfatal(traceback.format_exc())
            rospy.signal_shutdown("shutdown")
        except Exception as e:
            rospy.logerr("Failed to predict: %s" % str(e))
            rospy.logerr(traceback.format_exc())
            return

        for i in range(len(results)):
            results[i].update({
                "face_origin": face_origins[i],
                "face_2d_origin": face_2d_origins[i],
                "rect": rects[i],
            })

        self.publish_face_rects(header, results)
        self.publish_gender(header, results)
        self.publish_pose(header, results, img)

    def publish_face_rects(self, header, results):
        rects = RectArray(
            header=header,
            rects=[r["rect"] for r in results],
        )
        self.pub_rect.publish(rects)

    def publish_gender(self, header, results):
        target_names = ["Male", "Female"]
        labels = [0 if r["gender"] < 0.5 else 1 for r in results]
        msg = ClassificationResult(
            header=header,
            classifier=self.classifier_name,
            target_names=target_names,
            labels=labels,
            label_names=[target_names[l] for l in labels],
            label_proba=[r["detection"] for r in results],
        )
        self.pub_gender.publish(msg)

    def publish_pose(self, header, results, img):
        msg = PoseArray(header=header)
        for r in results:
            pose = r["face_origin"]
            pose_2d = r['face_2d_origin']
            ori = r["pose"]
            mat = T.euler_matrix(-ori[0], -ori[1], -ori[2])
            rotmat = mat[:3, :3]
            quat = T.quaternion_from_matrix(mat)
            quat = T.quaternion_multiply(
                [0.5, 0.5, -0.5, 0.5], quat)

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            msg.poses.append(pose)

            if self.visualize:
                zvec = np.array([0, 0, 1], np.float32)
                yvec = np.array([0, 1, 0], np.float32)
                xvec = np.array([1, 0, 0], np.float32)
                zvec = _project_plane_yz(rotmat.dot(zvec)) * 50.0
                yvec = _project_plane_yz(rotmat.dot(yvec)) * 50.0
                xvec = _project_plane_yz(rotmat.dot(xvec)) * 50.0

                face_2d_center = np.array([pose_2d.position.x, pose_2d.position.y])
                img = _draw_line(img, face_2d_center,
                                 face_2d_center + zvec, (255, 0, 0), 3)
                img = _draw_line(img, face_2d_center,
                                 face_2d_center + yvec, (0, 255, 0), 3)
                img = _draw_line(img, face_2d_center,
                                 face_2d_center + xvec, (0, 0, 255), 3)

        self.pub_pose.publish(msg)
        if self.visualize:
            img_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
            img_msg.header = header
            self.pub_debug_image.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node("face_pose_estimation")
    node = FacePoseEstimator()
    rospy.spin()
