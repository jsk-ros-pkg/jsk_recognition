#!/usr/bin/env python

from label_image_classifier import LabelImageClassifier
import numpy as np
import rospy


class ProbabilityImageClassifier(LabelImageClassifier):

    classifier_name = 'probability_image_classifier'

    def __init__(self):
        super(ProbabilityImageClassifier, self).__init__()

    def _classify(self, proba_img):
        proba = np.nansum(proba_img, axis=(0, 1)).astype(np.float32)

        self.ignore_labels = np.asarray(self.ignore_labels)

        # validation
        assert proba.ndim == 1
        n_labels = proba.shape[0]
        mask_valid = self.ignore_labels < n_labels
        if mask_valid.sum() != len(self.ignore_labels):
            rospy.logwarn_throttle(
                10, "The max label value in '~ignore_labels' exceeds "
                    "the number of labels of input probability image.")

        # filtering unrelated label probabilities
        ignore_labels = self.ignore_labels[mask_valid]
        proba[ignore_labels] = 0
        label = np.argmax(proba)
        proba = proba / proba.sum()

        return label, proba


if __name__ == '__main__':
    rospy.init_node('probability_image_classifier')
    app = ProbabilityImageClassifier()
    rospy.spin()
