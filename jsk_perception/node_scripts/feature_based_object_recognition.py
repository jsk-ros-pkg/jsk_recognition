#!/usr/bin/env python

import chainer
from chainer import cuda
import numpy as np
from sklearn.neighbors import KNeighborsClassifier

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_utils.chainermodels import ResNet152
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image


class ResNetFeature(ResNet152):

    def __call__(self, x):
        import chainer.functions as F
        h = self.bn1(self.conv1(x))
        h = F.max_pooling_2d(F.relu(h), 3, stride=2)
        h = self.res2(h)
        h = self.res3(h)
        h = self.res4(h)
        h = self.res5(h)
        h = F.average_pooling_2d(h, 7, stride=1)
        return h


class FeatureBasedObjectRecognition(ConnectionBasedTransport):

    def __init__(self):
        super(FeatureBasedObjectRecognition, self).__init__()
        # parameters
        pretrained_model = rospy.get_param('~pretrained_model')
        mean_file = rospy.get_param('~mean_file')
        self.gpu = rospy.get_param('~gpu', 0)
        # setup chainer
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False
        # model
        rospy.loginfo('Loading pretrained model')
        self.model = ResNetFeature()
        chainer.serializers.load_npz(pretrained_model, self.model)
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()
        rospy.loginfo('Finished loading pretrained model')
        # mean
        self.mean = np.load(mean_file)
        assert self.mean.shape == (224, 224, 3)  # BGR order
        # knn
        rospy.loginfo('Fitting KNN from db')
        db_file = rospy.get_param('~db_file')
        db = np.load(db_file)
        X, y, self.target_names = db['X'], db['y'], db['target_names']
        self.knn = KNeighborsClassifier(n_neighbors=10)
        self.knn.fit(X, y)
        rospy.loginfo('Finished fitting KNN from db')
        # setup publishers
        self.pub = self.advertise(
            '~output', ClassificationResult, queue_size=1)

    def subscribe(self):
        self.subs = []
        self.subs.append(message_filters.Subscriber('~input', Image))
        self.subs.append(message_filters.Subscriber('~input/mask', Image))
        queue_size = 100
        slop = 0.1
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs, queue_size=queue_size, slop=slop)
        self.sync.registerCallback(self.callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, imgmsg, mask_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
        mask = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='mono8')
        if mask.ndim == 3:
            mask = np.squeeze(mask, axis=2)
        mask = mask > 127  # uint8 -> bool

        img = img.astype(np.float64)
        img[mask] -= self.mean[mask]
        img[~mask] = 0

        img = img.transpose(2, 0, 1)
        img = img.astype(np.float32)
        x_data = np.asarray([img])
        if self.gpu >= 0:
            x_data = cuda.to_gpu(x_data)
        x = chainer.Variable(x_data)
        y = self.model(x)

        feat = cuda.to_cpu(y.data)
        feat = feat.squeeze(axis=(2, 3))
        X_query = feat

        y_pred_proba = self.knn.predict_proba(X_query)
        y_pred_proba = y_pred_proba.reshape(-1, 1, y_pred_proba.shape[1])
        y_pred_proba = y_pred_proba.mean(axis=1)
        y_pred = np.argmax(y_pred_proba, axis=1)

        classes = self.knn.classes_
        target_names = self.target_names[classes]

        msg = ClassificationResult()
        msg.header = imgmsg.header
        msg.labels = y_pred.tolist()
        msg.label_names = target_names[y_pred].tolist()
        msg.label_proba = y_pred_proba[:, y_pred].flatten().tolist()
        msg.probabilities = y_pred_proba.flatten().tolist()
        msg.target_names = target_names.tolist()
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('feature_based_object_recognition')
    app = FeatureBasedObjectRecognition()
    rospy.spin()
