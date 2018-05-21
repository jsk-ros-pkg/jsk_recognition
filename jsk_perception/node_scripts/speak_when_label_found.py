#!/usr/bin/env python

from __future__ import division

import os
import shutil
import tempfile

import cv_bridge
from jsk_gui_msgs.msg import SlackMessage
import message_filters
import rospy
from sensor_msgs.msg import Image
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


class SpeakWhenLabelFound(object):

    def __init__(self):
        self.tmpdir = tempfile.mkdtemp()

        self.pub_str = rospy.Publisher('~output/string', String, queue_size=1)
        self.pub_slack = rospy.Publisher(
            '~output/slack_msg', SlackMessage, queue_size=1)

        self.sound = None
        if rospy.get_param('~sound', True):
            self.sound = SoundClient(blocking=True)
            rospy.sleep(1)

        # Get target label id and label_names
        self.label_names = rospy.get_param('~label_names')
        self.target_labels = rospy.get_param('~target_labels', None)
        if self.target_labels is None:
            self.target_labels = []
            target_label_names = rospy.get_param('~target_label_names')
            for lname in target_label_names:
                self.target_labels.append(self.label_names.index(lname))
        self.min_label_region = rospy.get_param('~min_label_region', 0.1)

        self.sub_img = message_filters.Subscriber(
            '~input/image', Image, queue_size=1)
        self.sub_label = message_filters.Subscriber(
            '~input/label', Image, queue_size=1)
        queue_size = rospy.get_param('~queue_size', 100)
        if rospy.get_param('~approximate_sync', False):
            self.sync = message_filters.TimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=queue_size)
        else:
            slop = rospy.get_param('~slop', 0.1)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub_img, self.sub_label], queue_size=queue_size, slop=slop)
        self.sync.registerCallback(self.input_cb)

    def __del__(self):
        if os.path.exists(self.tmpdir):
            shutil.rmtree(self.tmpdir)

    def input_cb(self, imgmsg, label_msg):
        bridge = cv_bridge.CvBridge()
        label = bridge.imgmsg_to_cv2(label_msg)
        found_labels = []
        for lval in self.target_labels:
            label_region = (label == lval).sum() / label.size
            if label_region >= self.min_label_region:
                found_labels.append(lval)

        if not found_labels:
            rospy.loginfo('No label is found')
            return

        text = ' and '.join(self.label_names[lval] for lval in found_labels)
        text += ' is found'
        rospy.loginfo(text)
        self.pub_str.publish(String(data=text))
        if self.sound:
            self.sound.say(text)

        if self.pub_slack.get_num_connections() > 0:
            slack_msg = SlackMessage(text=text)
            if imgmsg:
                slack_msg.image = imgmsg
            self.pub_slack.publish(slack_msg)


if __name__ == '__main__':
    rospy.init_node('speak_when_label_found')
    SpeakWhenLabelFound()
    rospy.spin()
