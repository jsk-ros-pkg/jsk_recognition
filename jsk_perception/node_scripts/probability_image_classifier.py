#!/usr/bin/env python

from label_image_classifier import LabelImageClassifier
import numpy as np
import rospy


class ProbabilityImageClassifier(LabelImageClassifier):

    def __init__(self):
        super(ProbabilityImageClassifier, self).__init__()

    def _classify(self, proba_img):
        class_proba = proba_img.sum(axis=(0, 1)).astype(np.float32)
        class_proba[self.ignore_labels] = 0
        label = np.argmax(class_proba)
        label_proba = class_proba.max() / class_proba.sum()
        return label, label_proba


if __name__ == '__main__':
    rospy.init_node('probability_image_classifier')
    app = ProbabilityImageClassifier()
    rospy.spin()
