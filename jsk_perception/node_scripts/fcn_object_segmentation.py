#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import chainer
from chainer import cuda
import chainer.serializers as S
import fcn
import fcn.models

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import jsk_loginfo
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image


def softmax(w, t=1.0):
    e = np.exp(w)
    dist = e / np.sum(e, axis=0)
    return dist


class FCNObjectSegmentation(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self._load_model()
        self.pub = self.advertise('~output', Image, queue_size=1)
        self.pub_proba = self.advertise(
            '~output/proba_image', Image, queue_size=1)

    def _load_model(self):
        self.gpu = rospy.get_param('~gpu', -1)  # -1 is cpu mode
        self.model_name = rospy.get_param('~model_name')
        model_h5 = rospy.get_param('~model_h5')
        self.target_names = rospy.get_param('~target_names')
        self.mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])

        n_class = len(self.target_names)
        if self.model_name == 'fcn32s':
            self.model = fcn.models.FCN32s(n_class=n_class)
        elif self.model_name == 'fcn16s':
            self.model = fcn.models.FCN16s(n_class=n_class)
        elif self.model_name == 'fcn8s':
            self.model = fcn.models.FCN8s(n_class=n_class)
        else:
            rospy.logerr('Unsupported ~model_name: {0}'
                         .format(self.model_name))
        jsk_loginfo('Loading trained model: {0}'.format(model_h5))
        S.load_hdf5(model_h5, self.model)
        jsk_loginfo('Finished loading trained model: {0}'.format(model_h5))
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)

    def subscribe(self):
        use_mask = rospy.get_param('~use_mask', False)
        if use_mask:
            queue_size = rospy.get_param('~queue_size', 10)
            sub_img = message_filters.Subscriber(
                '~input', Image, queue_size=1, buff_size=2**24)
            sub_mask = message_filters.Subscriber(
                '~input/mask', Image, queue_size=1, buff_size=2**24)
            self.subs = [sub_img, sub_mask]
            if rospy.get_param('~approximate_sync', False):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    fs=self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self._cb_with_mask)
        else:
            # larger buff_size is necessary for taking time callback
            # http://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date/29160379#29160379  # NOQA
            sub_img = rospy.Subscriber(
                '~input', Image, self._cb, queue_size=1, buff_size=2**24)
            self.subs = [sub_img]

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb_with_mask(self, img_msg, mask_msg):
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        mask = br.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        label, proba_img = self.segment(img)
        label[mask == 0] = 0
        proba_img[:, :, 0][mask != 0] = 1
        proba_img[:, :, 1:][mask != 0] = 0
        label_msg = br.cv2_to_imgmsg(label.astype(np.int32), '32SC1')
        label_msg.header = img_msg.header
        self.pub.publish(label_msg)
        proba_msg = br.cv2_to_imgmsg(proba_img.astype(np.float32))
        proba_msg.header = img_msg.header
        self.pub_proba.publish(proba_msg)

    def _cb(self, img_msg):
        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        label, proba_img = self.segment(img)
        label_msg = br.cv2_to_imgmsg(label.astype(np.int32), '32SC1')
        label_msg.header = img_msg.header
        self.pub.publish(label_msg)
        proba_msg = br.cv2_to_imgmsg(proba_img.astype(np.float32))
        proba_msg.header = img_msg.header
        self.pub_proba.publish(proba_msg)

    def segment(self, bgr):
        blob = (bgr - self.mean_bgr).transpose((2, 0, 1))
        x_data = np.array([blob], dtype=np.float32)
        if self.gpu != -1:
            x_data = cuda.to_gpu(x_data, device=self.gpu)
        x = chainer.Variable(x_data, volatile=True)
        self.model.train = False
        self.model(x)
        pred = self.model.score
        score = cuda.to_cpu(pred.data)[0]
        proba_img = softmax(score).transpose((1, 2, 0))
        label = np.argmax(score, axis=0)
        return label, proba_img


if __name__ == '__main__':
    rospy.init_node('fcn_object_segmentation')
    FCNObjectSegmentation()
    rospy.spin()
