#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
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
from chainer import cuda
import chainer.serializers as S
from chainer import Variable
from distutils.version import LooseVersion
import numpy as np
import skimage.transform

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_utils.chainermodels import VGG16
from jsk_recognition_utils.chainermodels import VGG16BatchNormalization
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logerr_throttle
import message_filters
import rospy
from sensor_msgs.msg import Image


class VGG16ObjectRecognition(ConnectionBasedTransport):

    mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])

    def __init__(self):
        super(self.__class__, self).__init__()
        self.insize = 224
        self.gpu = rospy.get_param('~gpu', -1)
        self.target_names = rospy.get_param('~target_names')
        self.model_name = rospy.get_param('~model_name')
        if self.model_name == 'vgg16':
            self.model = VGG16(n_class=len(self.target_names))
        elif self.model_name == 'vgg16_batch_normalization':
            self.model = VGG16BatchNormalization(
                n_class=len(self.target_names))
        else:
            rospy.logerr('Unsupported ~model_name: {0}'
                         .format(self.model_name))
        model_file = rospy.get_param('~model_file')
        S.load_hdf5(model_file, self.model)
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)
        self.pub = self.advertise('~output', ClassificationResult,
                                  queue_size=1)
        self.pub_input = self.advertise(
            '~debug/net_input', Image, queue_size=1)

    def subscribe(self):
        if rospy.get_param('~use_mask', False):
            # larger buff_size is necessary for taking time callback
            # http://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date/29160379#29160379  # NOQA
            sub = message_filters.Subscriber(
                '~input', Image, queue_size=1, buff_size=2**24)
            sub_mask = message_filters.Subscriber(
                '~input/mask', Image, queue_size=1, buff_size=2**24)
            self.subs = [sub, sub_mask]
            queue_size = rospy.get_param('~queue_size', 10)
            if rospy.get_param('~approximate_sync', False):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    self.subs, queue_size=queue_size)
            sync.registerCallback(self._recognize)
        else:
            sub = rospy.Subscriber(
                '~input', Image, self._recognize, callback_args=None,
                queue_size=1, buff_size=2**24)
            self.subs = [sub]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _recognize(self, imgmsg, mask_msg=None):
        bridge = cv_bridge.CvBridge()
        bgr = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
        if mask_msg is not None:
            mask = bridge.imgmsg_to_cv2(mask_msg)
            if mask.shape != bgr.shape[:2]:
                logerr_throttle(10,
                                'Size of input image and mask is different')
                return
            elif mask.size == 0:
                logerr_throttle(10, 'Size of input mask is 0')
                return
            bgr[mask == 0] = self.mean_bgr
        bgr = skimage.transform.resize(
            bgr, (self.insize, self.insize), preserve_range=True)
        input_msg = bridge.cv2_to_imgmsg(bgr.astype(np.uint8), encoding='bgr8')
        input_msg.header = imgmsg.header
        self.pub_input.publish(input_msg)

        blob = (bgr - self.mean_bgr).transpose((2, 0, 1))
        x_data = np.array([blob], dtype=np.float32)
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

        proba = cuda.to_cpu(self.model.pred.data)[0]
        label = np.argmax(proba)
        label_name = self.target_names[label]
        label_proba = proba[label]
        cls_msg = ClassificationResult(
            header=imgmsg.header,
            labels=[label],
            label_names=[label_name],
            label_proba=[label_proba],
            probabilities=proba,
            classifier=self.model_name,
            target_names=self.target_names,
        )
        self.pub.publish(cls_msg)


if __name__ == '__main__':
    rospy.init_node('vgg16_object_recognition')
    app = VGG16ObjectRecognition()
    rospy.spin()
