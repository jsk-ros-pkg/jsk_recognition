#!/usr/bin/env python

import cv_bridge
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image

from jsk_perception.label_image_classifier import LabelImageClassifier



if __name__ == '__main__':
    rospy.init_node('label_image_classifier')
    app = LabelImageClassifier()
    rospy.spin()
