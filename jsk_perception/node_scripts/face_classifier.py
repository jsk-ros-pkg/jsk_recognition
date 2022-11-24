#!/usr/bin/env python

from jsk_recognition_utils.face_recognizer import FaceRecognizer

import rospy
import cv_bridge
from sensor_msgs.msg import Image


class FaceClassifier:

    def __init__(self):

        known_person_image_dir = rospy.get_param('~known_person_image_dir')
        fitting_method = rospy.get_param('~fitting_method', 'svm')
        self.recognizer = FaceRecognizer(known_person_image_dir, fitting_method)

        self.cv_bridge = cv_bridge.CvBridge()

        self.sub = rospy.Subscribe('~input', Image, self,callback)

    def callback(self, msg):

        image = self.cv_bridge.imgmsg_to_cv2(msg)
        names, locations = self.recognizer(image)
        rospy.loginfo('names: {}'.format(names))
        rospy.loginfo('locations: {}'.format(locations))


if __name__ == '__main__':

    rospy.init_node('face_classifier')
    node = FaceClassifier()
    rospy.spin()
