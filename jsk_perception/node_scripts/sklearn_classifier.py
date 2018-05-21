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
from jsk_recognition_msgs.msg import VectorArray, ClassificationResult


class ScikitLearnClassifier(ConnectionBasedTransport):
    def __init__(self):
        super(ScikitLearnClassifier, self).__init__()
        self._init_classifier()
        self._pub = self.advertise('~output', ClassificationResult,
                                   queue_size=1)

    def _init_classifier(self):
        clf_path = rospy.get_param('~clf_path')
        with gzip.open(clf_path) as f:
            self.clf = pickle.load(f)

    def subscribe(self):
        self.sub_hist = rospy.Subscriber('~input', VectorArray, self._predict)

    def unsubscribe(self):
        self.sub_hist.unregister()

    def _predict(self, msg):
        X = np.array(msg.data).reshape((-1, msg.vector_dim))
        normalize(X, copy=False)
        y_proba = self.clf.predict_proba(X)
        y_pred = np.argmax(y_proba, axis=-1)
        target_names = np.array(self.clf.target_names_)
        label_proba = [p[i] for p, i in zip(y_proba, y_pred)]
        out_msg = ClassificationResult(
            header=msg.header,
            labels=y_pred,
            label_names=target_names[y_pred],
            label_proba=label_proba,
            probabilities=y_proba.reshape(-1),
            classifier=self.clf.__str__(),
            target_names=target_names,
        )
        self._pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('sklearn_classifier')
    sklearn_clf = ScikitLearnClassifier()
    rospy.spin()
