#!/usr/bin/env python

from label_image_classifier import LabelImageClassifier
import numpy as np
import rospy


class ProbabilityImageClassifier(LabelImageClassifier):

    classifier_name = 'probability_image_classifier'

    def __init__(self):
        super(ProbabilityImageClassifier, self).__init__()

    def _classify(self, proba_img):
        proba = proba_img.sum(axis=(0, 1)).astype(np.float32)
        proba[self.ignore_labels] = 0
        label = np.argmax(proba)
        proba = proba / proba.sum()
        return label, proba


if __name__ == '__main__':
    rospy.init_node('probability_image_classifier')
    app = ProbabilityImageClassifier()
    rospy.spin()
