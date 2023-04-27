#!/usr/bin/env python

import cv_bridge
import rospy
from sensor_msgs.msg import Image

from jsk_recognition_msgs.msg import Label, LabelArray, Rect, RectArray
from jsk_recognition_utils.face_recognizer import FaceRecognizer


class FaceClassifier:

    def __init__(self):

        known_person_image_dir = rospy.get_param('~known_person_image_dir')
        fitting_method = rospy.get_param('~fitting_method', 'svm')
        self.recognizer = FaceRecognizer(
            known_person_image_dir, fitting_method)

        self.cv_bridge = cv_bridge.CvBridge()

        self.sub = rospy.Subscriber('~input', Image, self, callback)

        self.pub_rects = rospy.Publisher(
            '~output/rects', RectArray, queue_size=1)
        self.pub_labels = rospy.Publisher(
            '~output/labels', LabelArray, queue_size=1)

    def callback(self, msg):

        image = self.cv_bridge.imgmsg_to_cv2(msg)
        names, locations = self.recognizer(image)

        msg_rects = RectArray()
        msg_rects.header = msg.header
        msg_rects.rects = [Rect(x=loc[0], y=loc[1], width=loc[2]-loc[0], height=loc[3]-loc[1]) for loc in locations]
        msg_labels=LabelArray()
        msg_labels.header = msg.header
        msg_labels.labels = [Label(id=-1, name=name) for name in names]

        self.pub_rects.publish(msg_rects)
        self.pub_labels.publish(msg_labels)


if __name__ == '__main__':

    rospy.init_node('face_classifier')
    node=FaceClassifier()
    rospy.spin()
