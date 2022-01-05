#!/usr/bin/env python

# Copied from https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_perception/node_scripts/vgg16_object_recognition.py

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import chainer
from chainer import cuda
import chainer.serializers as S
from chainer import Variable
from distutils.version import LooseVersion
import numpy as np
import skimage.transform

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from sound_classification.nin.nin import NIN
from sound_classification.vgg16.vgg16_batch_normalization import VGG16BatchNormalization
from jsk_topic_tools import ConnectionBasedTransport  # TODO use LazyTransport
import os.path as osp
from sound_classification.process_gray_image import img_jet
import rospy
from sensor_msgs.msg import Image

from train import PreprocessedDataset


class SoundClassifier(ConnectionBasedTransport):
    """
    Classify spectrogram using neural network
    input: sensor_msgs/Image, 8UC1
    output jsk_recognition_msgs/ClassificationResult
    """

    def __init__(self):
        super(self.__class__, self).__init__()
        self.gpu = rospy.get_param('~gpu', -1)
        self.dataset = PreprocessedDataset()
        self.target_names_ordered = self.dataset.target_classes
        self.target_names = rospy.get_param('~target_names', self.target_names_ordered)
        for i, name in enumerate(self.target_names):
            if not name.endswith('\n'):
                self.target_names[i] = name + '\n'
        self.model_name = rospy.get_param('~model_name')
        if self.model_name == 'nin':
            self.insize = 227
            self.model = NIN(n_class=len(self.target_names))
        elif self.model_name == 'vgg16':
            self.insize = 224
            self.model = VGG16BatchNormalization(
                n_class=len(self.target))
        else:
            rospy.logerr('Unsupported ~model_name: {0}'
                         .format(self.model_name))
        model_file = rospy.get_param(
            '~model_file',
            osp.join(self.dataset.root, 'result',
                     self.model_name, 'model_best.npz'))
        S.load_npz(model_file, self.model)
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)
        self.pub = self.advertise('~output', ClassificationResult,
                                  queue_size=1)
        self.pub_input = self.advertise(
            '~debug/net_input', Image, queue_size=1)

    def subscribe(self):
        sub = rospy.Subscriber(
            '~input', Image, self._recognize, callback_args=None,
            queue_size=1, buff_size=2**24)
        self.subs = [sub]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _recognize(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        mono = bridge.imgmsg_to_cv2(imgmsg)
        bgr = img_jet(mono)
        bgr = skimage.transform.resize(
            bgr, (self.insize, self.insize), preserve_range=True)
        input_msg = bridge.cv2_to_imgmsg(bgr.astype(np.uint8), encoding='bgr8')
        input_msg.header = imgmsg.header
        self.pub_input.publish(input_msg)

        # (Height, Width, Channel) -> (Channel, Height, Width)
        # ###
        # print(type())
        # print(rgb.shape)
        # import cv2
        # cv2.imwrite('/home/naoya/test.png', rgb)
        # ###
        rgb = bgr.transpose((2, 0, 1))[::-1, :, :]
        rgb = self.dataset.process_image(rgb)
        x_data = np.array([rgb], dtype=np.float32)
        if self.gpu != -1:
            x_data = cuda.to_gpu(x_data, device=self.gpu)
        if LooseVersion(chainer.__version__) < LooseVersion('2.0.0'):
            x = Variable(x_data, volatile=True)
            self.model.train = False
            self.model(x)
        else:
            with chainer.using_config('train', False), \
                    chainer.no_backprop_mode():
                x = Variable(x_data)
                self.model(x)

        # swap_labels[label number in self.target_names]
        # -> label number in self.target_names_ordered
        swap_labels = [self.target_names_ordered.index(name) for name in self.target_names]
        for i in range(len(swap_labels)):
            if not (i in swap_labels):
                rospy.logerr('Wrong target_names is given by rosparam.')
                exit()
        proba = cuda.to_cpu(self.model.pred.data)[0]
        proba_swapped = [proba[swap_labels[i]] for i, p in enumerate(proba)]
        label_swapped = np.argmax(proba_swapped)
        label_name = self.target_names[label_swapped]
        label_proba = proba_swapped[label_swapped]
        cls_msg = ClassificationResult(
            header=imgmsg.header,
            labels=[label_swapped],
            label_names=[label_name],
            label_proba=[label_proba],
            probabilities=proba_swapped,
            classifier=self.model_name,
            target_names=self.target_names,
        )
        self.pub.publish(cls_msg)


if __name__ == '__main__':
    rospy.init_node('sound_classifier')
    app = SoundClassifier()
    rospy.spin()
