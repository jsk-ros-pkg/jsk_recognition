#!/usr/bin/env python

from __future__ import print_function

from distutils.version import LooseVersion

import itertools, pkg_resources, sys
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
from chainer import cuda
import chainer.functions as F
import chainer.serializers as S
import cv2
import numpy as np

import cv_bridge
from jsk_recognition_utils.chainermodels import FCN8sDepthPrediction
from jsk_recognition_utils.chainermodels import FCN8sDepthPredictionConcatFirst
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image


def colorize_depth(depth, min_value=None, max_value=None):
    min_value = np.nanmin(depth) if min_value is None else min_value
    max_value = np.nanmax(depth) if max_value is None else max_value
    if np.isinf(min_value) or np.isinf(max_value):
        rospy.logwarn('Min or max value for depth colorization is inf.')

    colorized = depth.copy()
    nan_mask = np.isnan(colorized)
    colorized[nan_mask] = 0
    colorized = 255 * (colorized - min_value) / (max_value - min_value)
    colorized = np.minimum(np.maximum(colorized, 0), 255).astype(np.uint8)
    colorized = cv2.applyColorMap(colorized, cv2.COLORMAP_JET)
    colorized[nan_mask] = (0, 0, 0)
    return colorized


class FCNDepthPrediction(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.backend = rospy.get_param('~backend', 'chainer')
        self.model_name = rospy.get_param('~model_name')
        self.model_file = rospy.get_param('~model_file')
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        self.target_names = rospy.get_param('~target_names')
        self.bg_label = rospy.get_param('~bg_label', 0)
        self.proba_threshold = rospy.get_param('~proba_threshold', 0.0)
        self.mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])
        self._load_model()
        self.pub_depth = self.advertise('~output', Image, queue_size=1)
        self.pub_depth_raw = self.advertise(
            '~output/depth_pred_raw', Image, queue_size=1)
        self.pub_label = self.advertise('~output/label', Image, queue_size=1)
        self.pub_proba = self.advertise(
            '~output/proba_image', Image, queue_size=1)

    def _load_model(self):
        if self.backend == 'chainer':
            self._load_chainer_model()
        else:
            raise RuntimeError('Unsupported backend: %s', self.backend)

    def _load_chainer_model(self):
        n_class = len(self.target_names)
        if self.model_name == 'fcn8s_depth_prediction':
            self.model = FCN8sDepthPrediction(n_class=n_class)
        elif self.model_name == 'fcn8s_depth_prediction_concat_first':
            self.model = FCN8sDepthPredictionConcatFirst(n_class=n_class)
        else:
            raise ValueError(
                'Unsupported ~model_name: {}'.format(self.model_name))
        rospy.loginfo('Loading trained model: {0}'.format(self.model_file))
        if self.model_file.endswith('.npz'):
            S.load_npz(self.model_file, self.model)
        rospy.loginfo(
            'Finished loading trained model: {0}'.format(self.model_file))
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)
        if LooseVersion(chainer.__version__) < LooseVersion('2.0.0'):
            self.model.train = False

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_img = message_filters.Subscriber(
            '~input', Image, queue_size=1, buff_size=2**24)
        sub_depth = message_filters.Subscriber(
            '~input/depth', Image, queue_size=1, buff_size=2**24)
        self.subs = [sub_img, sub_depth]
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

    def transform_depth(self, depth):
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        min_value = self.model.min_depth
        max_value = self.model.max_depth
        depth_viz_rgb = colorize_depth(
            depth,
            min_value=min_value, max_value=max_value
        )
        depth_viz_bgr = depth_viz_rgb[:, :, ::-1].astype(np.float32)
        depth_viz_bgr = (depth_viz_bgr - self.mean_bgr).transpose((2, 0, 1))
        return depth_viz_bgr

    def _cb(self, img_msg, depth_msg):
        br = cv_bridge.CvBridge()
        bgr_img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth_img = br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_img.ndim > 2:
            depth_img = np.squeeze(depth_img, axis=2)
        bgr_img = (bgr_img - self.mean_bgr).transpose((2, 0, 1))
        depth_viz_bgr = self.transform_depth(depth_img)

        label_pred, proba_img, depth_pred = \
            self.predict_depth(bgr_img, depth_viz_bgr)
        depth_pred_raw = depth_pred.copy()
        depth_pred[label_pred == 0] = depth_img[label_pred == 0]

        label_msg = br.cv2_to_imgmsg(label_pred.astype(np.int32), '32SC1')
        label_msg.header = img_msg.header
        self.pub_label.publish(label_msg)
        proba_msg = br.cv2_to_imgmsg(proba_img.astype(np.float32))
        proba_msg.header = img_msg.header
        self.pub_proba.publish(proba_msg)
        depth_msg = br.cv2_to_imgmsg(depth_pred.astype(np.float32))
        depth_msg.header = img_msg.header
        self.pub_depth.publish(depth_msg)
        depth_raw_msg = br.cv2_to_imgmsg(depth_pred_raw.astype(np.float32))
        depth_raw_msg.header = img_msg.header
        self.pub_depth_raw.publish(depth_raw_msg)

    def predict_depth(self, bgr, depth_bgr=None):
        if self.backend == 'chainer':
            return self._predict_depth_chainer_backend(bgr, depth_bgr)
        raise ValueError('Unsupported backend: {0}'.format(self.backend))

    def _predict_depth_chainer_backend(self, bgr, depth_bgr=None):
        bgr_data = np.array([bgr], dtype=np.float32)
        depth_bgr_data = np.array([depth_bgr], dtype=np.float32)
        if self.gpu != -1:
            bgr_data = cuda.to_gpu(bgr_data, device=self.gpu)
            depth_bgr_data = cuda.to_gpu(depth_bgr_data, device=self.gpu)
        if LooseVersion(chainer.__version__) < LooseVersion('2.0.0'):
            bgr = chainer.Variable(bgr_data, volatile=True)
            depth_bgr = chainer.Variable(depth_bgr_data, volatile=True)
            self.model(bgr, depth_bgr)
        else:
            with chainer.using_config('train', False):
                with chainer.no_backprop_mode():
                    bgr = chainer.Variable(bgr_data)
                    depth_bgr = chainer.Variable(depth_bgr_data)
                    self.model(bgr, depth_bgr)

        proba_img = F.softmax(self.model.mask_score)
        label_pred = F.argmax(self.model.mask_score, axis=1)
        depth_pred = F.sigmoid(self.model.depth_score)
        proba_img = F.transpose(proba_img, (0, 2, 3, 1))
        max_proba_img = F.max(proba_img, axis=-1)
        # squeeze batch axis, gpu -> cpu
        proba_img = cuda.to_cpu(proba_img.data)[0]
        max_proba_img = cuda.to_cpu(max_proba_img.data)[0]
        label_pred = cuda.to_cpu(label_pred.data)[0]
        depth_pred = cuda.to_cpu(depth_pred.data)[0]
        # uncertain because the probability is low
        label_pred[max_proba_img < self.proba_threshold] = self.bg_label
        # get depth image
        depth_pred = depth_pred[0, :, :]
        depth_pred *= (self.model.max_depth - self.model.min_depth)
        depth_pred += self.model.min_depth

        return label_pred, proba_img, depth_pred


if __name__ == '__main__':
    rospy.init_node('fcn_depth_prediction')
    FCNDepthPrediction()
    rospy.spin()
