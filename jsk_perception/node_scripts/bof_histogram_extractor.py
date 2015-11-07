#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Extract BoF histogram in realtime.
"""

import gzip
import cPickle as pickle

import numpy as np
from sklearn.preprocessing import normalize

import cv_bridge
from jsk_recognition_msgs.msg import VectorArray
from jsk_recognition_utils import decompose_descriptors_with_label
from jsk_topic_tools import jsk_logdebug, jsk_loginfo, ConnectionBasedTransport
import message_filters
from posedetection_msgs.msg import Feature0D
from sensor_msgs.msg import Image
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

        self._pub = self.advertise('~output', VectorArray, queue_size=1)
        jsk_loginfo('Initialized BoF histogram extractor')

    def subscribe(self):
        self._sub_feature = message_filters.Subscriber('~input', Feature0D)
        self._sub_label = message_filters.Subscriber('~input/label', Image)
        use_async = rospy.get_param('~approximate_sync', False)
        if use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                [self._sub_feature, self._sub_label], queue_size=10, slop=0.1)
        else:
            sync = message_filters.TimeSynchronizer(
                [self._sub_feature, self._sub_feature], queue_size=10)
        jsk_logdebug('~approximate_sync: {}'.format(use_async))
        sync.registerCallback(self._apply)

    def unsubscribe(self):
        self._sub_feature.unregister()
        self._sub_label.unregister()

    def _apply(self, feature_msg, label_msg):
        bridge = cv_bridge.CvBridge()
        label = bridge.imgmsg_to_cv2(label_msg)
        desc = np.array(feature_msg.descriptors)
        desc = desc.reshape((-1, feature_msg.descriptor_dim))
        pos = np.array(feature_msg.positions)
        pos = pos.reshape((-1, 2))
        decomposed = decompose_descriptors_with_label(
            descriptors=desc, positions=pos, label_img=label,
            skip_zero_label=True)
        X = self.bof.transform(np.array(decomposed.values()))
        normalize(X, copy=False)
        self._pub.publish(
            VectorArray(
                header=feature_msg.header,
                vector_dim=X.shape[1],
                data=X.reshape(-1),
            ))


if __name__ == '__main__':
    rospy.init_node('bof_histogram_extractor')
    extractor = BoFHistogramExtractor()
    rospy.spin()
