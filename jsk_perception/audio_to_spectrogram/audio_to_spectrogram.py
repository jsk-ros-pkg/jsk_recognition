#!/usr/bin/env python

from cv_bridge import CvBridge
import numpy as np

import rospy

from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image


# This node publish spectrogram (sensor_msgs/Image)
# from audio (audio_common_msgs/AudioData)
# The number of channel is assumed to be 1.
# The format of /audio is assumed to be wave.

class AudioToSpectrogram(object):

    def __init__(self):
        super(AudioToSpectrogram, self).__init__()

        # Audio topic config
        # Sampling rate of microphone (namely audio topic).
        mic_sampling_rate = rospy.get_param('~mic_sampling_rate', 16000)
        # Period[s] to sample audio data for one fft
        fft_sampling_period = rospy.get_param('~fft_sampling_period', 0.3)
        # Bits per one audio data
        bitdepth = rospy.get_param('~bitdepth', 16)
        if bitdepth == 16:
            self.dtype = 'int16'
        else:
            rospy.logerr("'~bitdepth' {} is unsupported.".format(bitdepth))
        # Audio topic buffer
        self.audio_buffer = np.array([], dtype=self.dtype)
        # How long audio_buffer should be for one audio topic
        self.audio_buffer_len = int(mic_sampling_rate * fft_sampling_period)

        # fft config
        window_function = np.arange(
            0.0, 1.0, 1.0 / self.audio_buffer_len)
        self.window_function = 0.54 - 0.46 * np.cos(
            2 * np.pi * window_function)
        high_cut_freq = rospy.get_param('~high_cut_freq', 800)
        if high_cut_freq > mic_sampling_rate / 2:
            rospy.logerr('Set high_cut_freq lower than {} Hz'.format(
                mic_sampling_rate / 2))
        low_cut_freq = rospy.get_param('~low_cut_freq', 0)
        freq = np.fft.fftfreq(
            self.audio_buffer_len, d=1./mic_sampling_rate)
        self.cutoff_mask = np.where(
            (low_cut_freq <= freq) & (freq <= high_cut_freq),
            True, False)
        band_pass_freq = freq[self.cutoff_mask]
        # How many times fft is executed in one second
        # fft_exec_rate equals to output spectrogram hz
        self.fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)

        # Spectrogram config
        # Period[s] to store audio data to create one spectrogram topic
        spectrogram_period = rospy.get_param('~spectrogram_period', 5)
        # self.spectrogram_len equals to horizontal pixel of output spectrogram
        # equals to fft_exec_rate * spectrogram_period
        self.spectrogram_len = int(
            self.fft_exec_rate * spectrogram_period)
        self.spectrogram = np.zeros((
            0,
            len(band_pass_freq)))
        rospy.logwarn('Please wait for the first spectrogram to come.')

        # ROS
        rospy.Subscriber(
            '~audio', AudioData, self.audio_cb)
        self.pub_spectrogram = rospy.Publisher(
            '~spectrogram', Image, queue_size=1)
        rospy.Timer(rospy.Duration(1. / self.fft_exec_rate), self.timer_cb)
        self.bridge = CvBridge()

    def audio_cb(self, msg):
        # Save audio msg to audio_buffer
        data = msg.data
        audio_buffer = np.frombuffer(data, dtype=self.dtype)
        self.audio_buffer = np.append(
            self.audio_buffer, audio_buffer)
        self.audio_buffer = self.audio_buffer[
            -self.audio_buffer_len:]

    def timer_cb(self, timer):
        if len(self.audio_buffer) != self.audio_buffer_len:
            return
        # Calc spectrogram by fft
        spectrum = np.fft.fft(self.audio_buffer * self.window_function)
        spectrum = np.log(np.abs(spectrum))
        spectrum = spectrum[self.cutoff_mask]
        # Create and publish spectrogram
        self.spectrogram = np.concatenate(
            [self.spectrogram, spectrum[None]])
        if self.spectrogram.shape[0] < self.spectrogram_len:
            return
        self.spectrogram = self.spectrogram[-self.spectrogram_len:].astype(
            np.float32)
        spectrogram_pub = self.spectrogram.transpose(1, 0)[::-1, :]
        spectrogram_pub = self.bridge.cv2_to_imgmsg(spectrogram_pub, '32FC1')
        spectrogram_pub.header.stamp = rospy.Time.now()
        self.pub_spectrogram.publish(spectrogram_pub)


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrogram')
    AudioToSpectrogram()
    rospy.spin()
