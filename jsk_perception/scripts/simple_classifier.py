#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import division
import gzip
import cPickle as pickle

import numpy as np
from sklearn.preprocessing import normalize

import rospy
from jsk_recognition_msgs.msg import Histogram
from std_msgs.msg import String


class SimpleClassifier(object):
    def __init__(self, clf_path):
        self._init_classifier(clf_path)
        self.input = Histogram()
        self._pub = rospy.Publisher('~output', String, queue_size=1)
        rospy.Subscriber('~input', Histogram, self._cb_predict)

    def _init_classifier(self, clf_path):
        with gzip.open(clf_path) as f:
            self.clf = pickle.load(f)

    def _cb_predict(self, msg):
        clf = self.clf
        if not (len(msg.histogram) > 0):
            return
        X = np.array([msg.histogram])
        normalize(X, copy=False)
        target_names = clf.target_names_
        index = clf.predict(X)[0]
        y_pred_0 = target_names[index]
        self._pub.publish(String(data=str(y_pred_0)))


def main():
    rospy.init_node('simple_classifier')
    clf_path = rospy.get_param('~clf_path')
    clf = SimpleClassifier(clf_path=clf_path)
    rospy.spin()


if __name__ == '__main__':
    main()
