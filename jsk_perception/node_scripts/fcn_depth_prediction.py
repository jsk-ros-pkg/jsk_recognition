#!/usr/bin/env python

from distutils.version import LooseVersion

import chainer
from chainer import cuda
import chainer.functions as F
import chainer.links as L
import chainer.serializers as S

import fcn
import matplotlib.cm
import numpy as np

import cv_bridge
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
    colorized = 1. * (colorized - min_value) / (max_value - min_value)
    colorized = matplotlib.cm.jet(colorized)[:, :, :3]
    colorized = (colorized * 255).astype(np.uint8)
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


class FCN8sDepthPrediction(chainer.Chain):

    # [0.2, 3]
    min_depth = 0.2
    max_depth = 3.

    def __init__(self, n_class, masking=True, concat=True):
        self.n_class = n_class
        self.masking = masking
        self.concat = concat
        kwargs = {
            'initialW': chainer.initializers.Zero(),
            'initial_bias': chainer.initializers.Zero(),
        }
        super(self.__class__, self).__init__()
        with self.init_scope():
            self.conv_rgb_1_1 = L.Convolution2D(3, 64, 3, 1, 100, **kwargs)
            self.conv_rgb_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_rgb_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_rgb_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_rgb_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_rgb_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_rgb_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.rgb_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)
            self.rgb_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.mask_score_fr = L.Convolution2D(
                4096, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore2 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.mask_upscore8 = L.Deconvolution2D(
                n_class, n_class, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.mask_score_pool3 = L.Convolution2D(
                256, n_class, 1, 1, 0, **kwargs)
            self.mask_score_pool4 = L.Convolution2D(
                512, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore_pool4 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.conv_depth_1_1 = L.Convolution2D(3, 64, 3, 1, 100, **kwargs)
            self.conv_depth_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_depth_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_depth_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_depth_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_depth_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_depth_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            if self.concat is True:
                self.concat_fc6 = L.Convolution2D(
                    1024, 4096, 7, 1, 0, **kwargs)
            else:
                self.depth_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)

            self.concat_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.depth_score_fr = L.Convolution2D(4096, 1, 1, 1, 0, **kwargs)

            self.depth_upscore2 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.depth_upscore8 = L.Deconvolution2D(
                1, 1, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.depth_score_pool3 = L.Convolution2D(
                256, 1, 1, 1, 0, **kwargs)
            self.depth_score_pool4 = L.Convolution2D(
                512, 1, 1, 1, 0, **kwargs)

            self.depth_upscore_pool4 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

    def predict_mask(self, rgb, return_pool5=False):
        # conv_rgb_1
        h = F.relu(self.conv_rgb_1_1(rgb))
        h = F.relu(self.conv_rgb_1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool1 = h  # 1/2

        # conv_rgb_2
        h = F.relu(self.conv_rgb_2_1(rgb_pool1))
        h = F.relu(self.conv_rgb_2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool2 = h  # 1/4

        # conv_rgb_3
        h = F.relu(self.conv_rgb_3_1(rgb_pool2))
        h = F.relu(self.conv_rgb_3_2(h))
        h = F.relu(self.conv_rgb_3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool3 = h  # 1/8

        # conv_rgb_4
        h = F.relu(self.conv_rgb_4_1(rgb_pool3))
        h = F.relu(self.conv_rgb_4_2(h))
        h = F.relu(self.conv_rgb_4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool4 = h  # 1/16

        # conv_rgb_5
        h = F.relu(self.conv_rgb_5_1(rgb_pool4))
        h = F.relu(self.conv_rgb_5_2(h))
        h = F.relu(self.conv_rgb_5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool5 = h  # 1/32

        # rgb_fc6
        h = F.relu(self.rgb_fc6(rgb_pool5))
        h = F.dropout(h, ratio=.5)
        rgb_fc6 = h  # 1/32

        # rgb_fc7
        h = F.relu(self.rgb_fc7(rgb_fc6))
        h = F.dropout(h, ratio=.5)
        rgb_fc7 = h  # 1/32

        # mask_score_fr
        h = self.mask_score_fr(rgb_fc7)
        mask_score_fr = h  # 1/32

        # mask_score_pool3
        scale_rgb_pool3 = 0.0001 * rgb_pool3
        h = self.mask_score_pool3(scale_rgb_pool3)
        mask_score_pool3 = h  # 1/8

        # mask_score_pool4
        scale_rgb_pool4 = 0.01 * rgb_pool4
        h = self.mask_score_pool4(scale_rgb_pool4)
        mask_score_pool4 = h  # 1/16

        # mask upscore2
        h = self.mask_upscore2(mask_score_fr)
        mask_upscore2 = h  # 1/16

        # mask_score_pool4c
        h = mask_score_pool4[:, :,
                             5:5 + mask_upscore2.data.shape[2],
                             5:5 + mask_upscore2.data.shape[3]]
        mask_score_pool4c = h  # 1/16

        # mask_fuse_pool4
        h = mask_upscore2 + mask_score_pool4c
        mask_fuse_pool4 = h  # 1/16

        # mask_upscore_pool4
        h = self.mask_upscore_pool4(mask_fuse_pool4)
        mask_upscore_pool4 = h  # 1/8

        # mask_score_pool3c
        h = mask_score_pool3[:, :,
                             9:9 + mask_upscore_pool4.data.shape[2],
                             9:9 + mask_upscore_pool4.data.shape[3]]
        mask_score_pool3c = h  # 1/8

        # mask_fuse_pool3
        h = mask_upscore_pool4 + mask_score_pool3c
        mask_fuse_pool3 = h  # 1/8

        # mask_upscore8
        h = self.mask_upscore8(mask_fuse_pool3)
        mask_upscore8 = h  # 1/1

        # mask_score
        h = mask_upscore8[:, :,
                          31:31 + rgb.shape[2], 31:31 + rgb.shape[3]]
        mask_score = h  # 1/1

        if return_pool5:
            return mask_score, rgb_pool5
        else:
            return mask_score

    def predict_depth(self, rgb, mask_score, depth_viz, rgb_pool5):
        # conv_depth_1
        h = F.relu(self.conv_depth_1_1(depth_viz))
        h = F.relu(self.conv_depth_1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool1 = h  # 1/2

        # conv_depth_2
        h = F.relu(self.conv_depth_2_1(depth_pool1))
        h = F.relu(self.conv_depth_2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool2 = h  # 1/4

        # conv_depth_3
        h = F.relu(self.conv_depth_3_1(depth_pool2))
        h = F.relu(self.conv_depth_3_2(h))
        h = F.relu(self.conv_depth_3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool3 = h  # 1/8

        # conv_depth_4
        h = F.relu(self.conv_depth_4_1(depth_pool3))
        h = F.relu(self.conv_depth_4_2(h))
        h = F.relu(self.conv_depth_4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool4 = h  # 1/16

        # conv_depth_5
        h = F.relu(self.conv_depth_5_1(depth_pool4))
        h = F.relu(self.conv_depth_5_2(h))
        h = F.relu(self.conv_depth_5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool5 = h  # 1/32

        if self.masking is True:
            # Apply negative_mask to depth_pool5
            # (N, C, H, W) -> (N, H, W)
            mask_pred_tmp = F.argmax(self.mask_score, axis=1)
            # (N, H, W) -> (N, 1, H, W), float required for resizing
            mask_pred_tmp = mask_pred_tmp[:, None, :, :].data.astype(
                self.xp.float32)  # 1/1
            resized_mask_pred = F.resize_images(
                mask_pred_tmp,
                (depth_pool5.shape[2], depth_pool5.shape[3]))  # 1/32
            depth_pool5_cp = depth_pool5
            masked_depth_pool5 = depth_pool5_cp * \
                (resized_mask_pred.data == 0.0).astype(self.xp.float32)
        else:
            masked_depth_pool5 = depth_pool5

        if self.concat is True:
            # concatenate rgb_pool5 and depth_pool5
            concat_pool5 = F.concat((rgb_pool5, masked_depth_pool5), axis=1)

            # concat_fc6
            h = F.relu(self.concat_fc6(concat_pool5))
            h = F.dropout(h, ratio=.5)
            concat_fc6 = h  # 1/32
        else:
            # concat_fc6
            h = F.relu(self.depth_fc6(masked_depth_pool5))
            h = F.dropout(h, ratio=.5)
            concat_fc6 = h  # 1/32

        # concat_fc7
        h = F.relu(self.concat_fc7(concat_fc6))
        h = F.dropout(h, ratio=.5)
        concat_fc7 = h  # 1/32

        # depth_score_fr
        h = self.depth_score_fr(concat_fc7)
        depth_score_fr = h  # 1/32

        # depth_score_pool3
        scale_depth_pool3 = 0.0001 * depth_pool3
        h = self.depth_score_pool3(scale_depth_pool3)
        depth_score_pool3 = h  # 1/8

        # depth_score_pool4
        scale_depth_pool4 = 0.01 * depth_pool4
        h = self.depth_score_pool4(scale_depth_pool4)
        depth_score_pool4 = h  # 1/16

        # depth upscore2
        h = self.depth_upscore2(depth_score_fr)
        depth_upscore2 = h  # 1/16

        # depth_score_pool4c
        h = depth_score_pool4[:, :,
                              5:5 + depth_upscore2.data.shape[2],
                              5:5 + depth_upscore2.data.shape[3]]
        depth_score_pool4c = h  # 1/16

        # depth_fuse_pool4
        h = depth_upscore2 + depth_score_pool4c
        depth_fuse_pool4 = h  # 1/16

        # depth_upscore_pool4
        h = self.depth_upscore_pool4(depth_fuse_pool4)
        depth_upscore_pool4 = h  # 1/8

        # depth_score_pool3c
        h = depth_score_pool3[:, :,
                              9:9 + depth_upscore_pool4.data.shape[2],
                              9:9 + depth_upscore_pool4.data.shape[3]]
        depth_score_pool3c = h  # 1/8

        # depth_fuse_pool3
        h = depth_upscore_pool4 + depth_score_pool3c
        depth_fuse_pool3 = h  # 1/8

        # depth_upscore8
        h = self.depth_upscore8(depth_fuse_pool3)
        depth_upscore8 = h  # 1/1

        # depth_score
        h = depth_upscore8[:, :,
                           31:31 + rgb.shape[2],
                           31:31 + rgb.shape[3]]
        depth_score = h  # 1/1

        return depth_score

    def __call__(self, rgb, depth_viz):
        mask_score, rgb_pool5 = self.predict_mask(rgb, return_pool5=True)
        self.mask_score = mask_score

        depth_score = self.predict_depth(
            rgb, mask_score, depth_viz, rgb_pool5)
        self.depth_score = depth_score

        assert not chainer.config.train
        return


if __name__ == '__main__':
    rospy.init_node('fcn_depth_prediction')
    FCNDepthPrediction()
    rospy.spin()
