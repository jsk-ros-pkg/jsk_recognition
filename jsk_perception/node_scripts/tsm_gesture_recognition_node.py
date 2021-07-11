#!/usr/bin/env python

from collections import deque

import cv2
from cv_bridge import CvBridge
import jsk_recognition_msgs.msg
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg


def jester_score_smoothing(scores, label_names):
    label2index = {label: index for index, label in enumerate(label_names)}
    scores[label2index['Rolling Hand Backward']] = 0.0
    scores[label2index['Rolling Hand Forward']] = 0.0
    scores[label2index['Turning Hand Clockwise']] = 0.0
    scores[label2index['Turning Hand Counterclockwise']] = 0.0
    scores[label2index['Pulling Hand In']] = 0.0
    scores[label2index['No gesture']] += \
        scores[label2index['Doing other things']]
    scores[label2index['Doing other things']] = 0.0
    return scores


def jester_history_smoothing(idx, history, max_hist_len=20):
    """History smoothing for jester gesture recognition dataset.

    Parameteres
    -----------
    idx : int
        label id.
    history : list[int]
        history of label ids.
    """
    # history smoothing
    if len(history) > 1 and idx != history[-1]:
        if not (history[-1] == history[-2]):
            idx = history[-1]

    history.append(idx)
    history = history[-max_hist_len:]
    return history[-1], history


class TSMGestureRecognitionNode(object):

    def __init__(self):
        super(TSMGestureRecognitionNode, self).__init__()

        self.score_history = deque(maxlen=12)
        self.label_history = []

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            '~output/viz',
            sensor_msgs.msg.Image,
            queue_size=1)
        self.pub_class = rospy.Publisher(
            "~output/class",
            jsk_recognition_msgs.msg.ClassificationResult,
            queue_size=1)

        self.subscribe()

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 30)
        sub_img = message_filters.Subscriber(
            '~input', sensor_msgs.msg.Image,
            queue_size=1, buff_size=2**24)
        sub_class = message_filters.Subscriber(
            '~input/class', jsk_recognition_msgs.msg.ClassificationResult,
            queue_size=1, buff_size=2**24)
        self.subs = [sub_img, sub_class]

        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def callback(self, img_msg, cls_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='rgb8')

        label_names = cls_msg.label_names
        scores = np.array(cls_msg.label_proba)
        scores = jester_score_smoothing(scores, label_names)
        scores -= scores.max()
        scores = np.exp(scores) / np.sum(np.exp(scores))

        self.score_history.append(scores)
        average_score = sum(self.score_history)
        idx = np.argmax(average_score)

        idx, self.label_history = jester_history_smoothing(
            idx, self.label_history)

        cls_msg.label_proba = scores
        self.pub_class.publish(cls_msg)

        if self.image_pub.get_num_connections() > 0:
            height, width, _ = rgb_img.shape
            viz_img = cv2.resize(rgb_img, (width, height))
            viz_img = cv2.cvtColor(viz_img, cv2.COLOR_RGB2BGR)
            label_img = 255 * np.ones((height // 10, width, 3)).astype('uint8')

            cv2.putText(label_img, 'Prediction: ' + label_names[idx],
                        (0, int(height / 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 2)
            viz_img = np.concatenate((viz_img, label_img), axis=0)
            msg = self.bridge.cv2_to_imgmsg(viz_img, "bgr8")
            msg.header = img_msg.header
            self.image_pub.publish(msg)


def main():
    rospy.init_node('tsm_gesture_recognition_node')
    act = TSMGestureRecognitionNode()  # NOQA
    rospy.spin()


if __name__ == '__main__':
    main()
