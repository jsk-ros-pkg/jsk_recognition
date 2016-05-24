#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from chainer import cuda
from chainer import Variable
import chainer.serializers as S
import numpy as np
import skimage.transform

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_utils.chainermodels import VGG16BatchNormalization
from jsk_recognition_msgs.msg import ClassificationResult
import rospy
from sensor_msgs.msg import Image


class VGG16ObjectRecognition(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.gpu = rospy.get_param('~gpu', -1)
        self.mean_bgr = np.array([104.00698793, 116.66876762, 122.67891434])
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
        model_h5 = rospy.get_param('~model_h5')
        S.load_hdf5(model_h5, self.model)
        if self.gpu != -1:
            self.model.to_gpu(self.gpu)
        self.pub = self.advertise('~output', ClassificationResult,
                                  queue_size=1)
        self.pub_input = self.advertise(
            '~debug/net_input', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._recognize)

    def unsubscribe(self):
        self.sub.unregister()

    def _recognize(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        bgr = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
        bgr = skimage.transform.resize(bgr, (224, 224), preserve_range=True)
        input_msg = bridge.cv2_to_imgmsg(bgr.astype(np.uint8), encoding='bgr8')
        input_msg.header = imgmsg.header
        self.pub_input.publish(input_msg)

        blob = (bgr - self.mean_bgr).transpose((2, 0, 1))
        x_data = np.array([blob], dtype=np.float32)
        if self.gpu != -1:
            x_data = cuda.to_gpu(x_data, device=self.gpu)
        x = Variable(x_data, volatile=True)

        self.model.train = False
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
