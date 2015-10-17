#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Extract BoF histogram in realtime.
"""

import gzip
import cPickle as pickle

import numpy as np
from sklearn.preprocessing import normalize

from jsk_recognition_msgs.msg import Histogram
from jsk_topic_tools import jsk_loginfo, ConnectionBasedTransport
from posedetection_msgs.msg import Feature0D
import rospy


class BoFHistogramExtractor(ConnectionBasedTransport):
    def __init__(self):
        super(BoFHistogramExtractor, self).__init__()

        # load bag of features
        jsk_loginfo('Loading BoF data')
        bof_data = rospy.get_param('~bof_data', None)
        if bof_data is None:
            quit()
        with gzip.open(bof_data, 'rb') as f:
            self.bof = pickle.load(f)

        self.pub = self.advertise('~output', Histogram, queue_size=1)
        jsk_loginfo('Initialized BoF histogram extractor')

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Feature0D, self._extract)

    def unsubscribe(self):
        self.sub.unregister()

    def _extract(self, msg):
        desc = np.array(msg.descriptors)
        X = self.bof.transform([desc])
        normalize(X, copy=False)
        self.pub.publish(Histogram(header=msg.header, histogram=X[0]))


if __name__ == '__main__':
    rospy.init_node('bof_histogram_extractor')
    extractor = BoFHistogramExtractor()
    rospy.spin()
