#!/usr/bin/env python

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image


class LabelImageClassifier(ConnectionBasedTransport):

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
        label, label_proba = self._classify(img)
        msg = ClassificationResult()
        msg.header = imgmsg.header
        msg.labels = [label]
        msg.label_names = [self.target_names[label]]
        msg.label_proba = [label_proba]
        self.pub.publish(msg)

    def _classify(self, label_img):
        label_img = label_img.flatten()
        counts = np.bincount(label_img)
        counts[self.ignore_labels] = 0
        label = np.argmax(counts)
        label_proba = counts.max().astype(np.float32) / len(label_img)
        return label, label_proba


if __name__ == '__main__':
    rospy.init_node('label_image_classifier')
    app = LabelImageClassifier()
    rospy.spin()
