#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import division
import gzip
import cPickle as pickle

import numpy as np
from sklearn.preprocessing import normalize

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import Histogram, ClassificationResult


class SimpleClassifier(ConnectionBasedTransport):
    def __init__(self):
        super(SimpleClassifier, self).__init__()
        self._init_classifier()
        self.input = Histogram()
        self._pub = self.advertise('~output', ClassificationResult,
                                   queue_size=1)

    def _init_classifier(self):
        clf_path = rospy.get_param('~clf_path')
        with gzip.open(clf_path) as f:
            self.clf = pickle.load(f)

    def subscribe(self):
        self.sub_hist = rospy.Subscriber('~input', Histogram, self._predict)

    def unsubscribe(self):
        self.sub_hist.unregister()

    def _predict(self, msg):
        if not (len(msg.histogram) > 0):
            return
        X = np.array([msg.histogram])
        normalize(X, copy=False)
        y_proba = self.clf.predict_proba(X)
        y_pred = np.argmax(y_proba, axis=-1)
        target_names = np.array(self.clf.target_names_)
        self._pub.publish(ClassificationResult(
            header=msg.header,
            labels=y_pred,
            label_names=target_names[y_pred],
            label_proba=y_proba[:, y_pred],
            probabilities=y_proba.reshape(-1),
            classifier=self.clf.__str__(),
            target_names=target_names,
        ))


if __name__ == '__main__':
    rospy.init_node('simple_classifier')
    simple_clf = SimpleClassifier()
    rospy.spin()
