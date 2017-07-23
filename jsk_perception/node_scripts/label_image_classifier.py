#!/usr/bin/env python

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image


class LabelImageClassifier(ConnectionBasedTransport):

    classifier_name = 'label_image_classifier'

    def __init__(self):
        super(LabelImageClassifier, self).__init__()
        self.ignore_labels = rospy.get_param('~ignore_labels', [])
        self.target_names = rospy.get_param('~target_names', [])
        self.pub = self.advertise(
            '~output', ClassificationResult, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg)
        label, proba = self._classify(img)
        msg = ClassificationResult()
        msg.header = imgmsg.header
        msg.labels = [label]
        msg.label_names = [self.target_names[label]]
        msg.label_proba = [proba[label]]
        msg.probabilities = proba
        msg.classifier = self.classifier_name
        msg.target_names = self.target_names
        self.pub.publish(msg)

    def _classify(self, label_img):
        counts = np.bincount(label_img.flatten(),
                             minlength=len(self.target_names))
        counts[self.ignore_labels] = 0
        label = np.argmax(counts)
        proba = counts.astype(np.float32) / counts.sum()
        return label, proba


if __name__ == '__main__':
    rospy.init_node('label_image_classifier')
    app = LabelImageClassifier()
    rospy.spin()
