#!/usr/bin/env python

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

        spectrum_msg = rospy.wait_for_message('~spectrum', Spectrum)
        spectrum_len = len(spectrum_msg.amplitude)

        # Spectrogram config
        # Period[s] to store audio data to create one spectrogram topic
        spectrogram_period = rospy.get_param('~spectrogram_period', 5)
        # How many times fft is executed in one second
        # fft_exec_rate equals to input spectrum hz and output spectrogram hz
        fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)
        # self.spectrogram_len equals to horizontal pixel of output spectrogram
        self.spectrogram_len = int(
            fft_exec_rate * spectrogram_period)
        self.spectrogram = np.zeros((0, spectrum_len),
                                    dtype=np.float32)
        rospy.logwarn('Please wait for the first spectrogram to come.')

        # ROS
        rospy.Subscriber(
            '~spectrum', Spectrum, self.audio_cb)
        self.pub_spectrogram = rospy.Publisher(
            '~spectrogram', Image, queue_size=1)
        rospy.Timer(rospy.Duration(1. / fft_exec_rate), self.timer_cb)
        self.bridge = CvBridge()

    def audio_cb(self, msg):
        # Create and publish spectrogram
        spectrum = np.array(msg.amplitude, dtype=np.float32)
        self.spectrogram = np.concatenate(
            [self.spectrogram, spectrum[None]])
        self.spectrogram = self.spectrogram[-self.spectrogram_len:]

    def timer_cb(self, timer):
        if self.spectrogram.shape[0] < self.spectrogram_len:
            return
        spectrogram = self.spectrogram.transpose(1, 0)[::-1, :]
        spectrogram_msg = self.bridge.cv2_to_imgmsg(spectrogram, '32FC1')
        spectrogram_msg.header.stamp = rospy.Time.now()
        self.pub_spectrogram.publish(spectrogram_msg)


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrogram')
    SpectrumToSpectrogram()
    rospy.spin()
