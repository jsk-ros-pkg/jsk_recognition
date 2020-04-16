#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy

from jsk_recognition_msgs.msg import Spectrum
from sensor_msgs.msg import Image


# This node publish spectrogram (sensor_msgs/Image)
# from spectrum (jsk_recognition_msgs/Spectrum)

class SpectrumToSpectrogram(object):

    def __init__(self):
        super(SpectrumToSpectrogram, self).__init__()
        # Set spectrogram shape
        self.image_height = rospy.get_param('~image_height', 300)
        self.image_width = rospy.get_param('~image_width', 300)
        # Get spectrum length
        spectrum_msg = rospy.wait_for_message('~spectrum', Spectrum)
        spectrum_len = len(spectrum_msg.amplitude)
        # Buffer for spectrum topic
        self.spectrogram = np.zeros((0, spectrum_len), dtype=np.float32)
        self.spectrum_stamp = []
        # Period[s] to store audio data to create one spectrogram topic
        self.spectrogram_period = rospy.get_param('~spectrogram_period', 5)
        # ROS subscriber and publisher
        rospy.Subscriber(
            '~spectrum', Spectrum, self.audio_cb)
        self.pub_spectrogram = rospy.Publisher(
            '~spectrogram', Image, queue_size=1)
        publish_rate = rospy.get_param(
            '~publish_rate', float(self.image_width / self.spectrogram_period))
        rospy.Timer(
            rospy.Duration(
                1.0 / publish_rate),
            self.timer_cb)
        self.bridge = CvBridge()

    def audio_cb(self, msg):
        # Add spectrum msg to buffer
        spectrum = np.array(msg.amplitude, dtype=np.float32)
        self.spectrogram = np.concatenate(
            [self.spectrogram, spectrum[None]])
        self.spectrum_stamp.append(msg.header.stamp)
        # Extract spectrogram of last (self.spectrogram_period) seconds
        time_now = rospy.Time.now()
        for i, stamp in enumerate(self.spectrum_stamp):
            if (time_now - stamp).to_sec() < self.spectrogram_period:
                self.spectrogram = self.spectrogram[i:]
                self.spectrum_stamp = self.spectrum_stamp[i:]
                break

    def timer_cb(self, timer):
        if self.spectrogram.shape[0] == 0:
            return
        # Reshape spectrogram
        spectrogram = self.spectrogram.transpose(1, 0)[::-1, :]
        spectrogram = cv2.resize(
            spectrogram, (self.image_width, self.image_height))
        # Publish spectrogram
        spectrogram_msg = self.bridge.cv2_to_imgmsg(spectrogram, '32FC1')
        spectrogram_msg.header.stamp = self.spectrum_stamp[-1]
        self.pub_spectrogram.publish(spectrogram_msg)


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrogram')
    SpectrumToSpectrogram()
    rospy.spin()
