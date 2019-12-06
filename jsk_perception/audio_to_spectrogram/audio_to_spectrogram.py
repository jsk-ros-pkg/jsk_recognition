#!/usr/bin/env python

from cv_bridge import CvBridge
import numpy as np

import rospy

from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image


# This node publish spectrogram (sensor_msgs/Image)
# from audio (audio_common_msgs/AudioData)
# Respeaker is assumed to be used.
# https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/respeaker_ros/scripts/respeaker_node.py  # NOQA

class AudioToSpectrogram(object):

    def __init__(self):
        super(AudioToSpectrogram, self).__init__()
        # Audio topic config
        # Sampling rate of audio topic. This depends on microphone.
        sampling_rate = rospy.get_param('~sampling_rate', 16000)
        # Period[s] to sample audio data for one fft
        sampling_period = rospy.get_param('~sampling_period', 0.5)
        # Bits per one audio data
        bitdepth = rospy.get_param('~bitdepth', 16)
        if bitdepth == 16:
            self.dtype = 'int16'
        else:
            rospy.logerr("'~bitdepth' {} is unsupported.".format(bitdepth))

        # Audio topic buffer
        # buffer for audio topic
        self.audio_buffer = np.array([], dtype=self.dtype)
        # how long audio_buffer should be for one audio topic
        self.audio_buffer_len = int(sampling_rate * sampling_period)

        # fft config
        # window function for fft
        window_function = np.arange(
            0.0, 1.0, 1.0 / self.audio_buffer_len)
        self.window_function = 0.54 - 0.46 * np.cos(
            2 * np.pi * window_function)
        high_cut_freq = rospy.get_param('~high_cut_freq', sampling_rate // 2)
        low_cut_freq = rospy.get_param('~low_cut_freq', 0)
        freq = np.fft.fftfreq(
            self.audio_buffer_len, d=1./sampling_rate)
        self.cutoff_mask = np.where(
            (low_cut_freq <= freq) & (freq <= high_cut_freq),
            True, False)
        band_pass_freq = freq[self.cutoff_mask]

        # Spectrogram config
        # how long we store audio topic to create one spectrogram topic
        spectrogram_duration = rospy.get_param('~spectrogram_duration', 10)
        # how many audio data in one audio topic. Same as chunk.
        # fft is executed per audio data of frame_per_buffer length
        frame_per_buffer = rospy.get_param('~frame_per_buffer', 1024)
        audio_topic_frequency = sampling_rate / float(frame_per_buffer)
        self.spectrogram_len = int(
            spectrogram_duration * audio_topic_frequency)
        self.spectrogram = np.zeros((
            self.spectrogram_len,
            len(band_pass_freq)))

        # ROS
        rospy.Rate(100)
        self.sub_audio = rospy.Subscriber(
            '~audio', AudioData, self._cb, queue_size=1000, buff_size=2**24)
        self.pub_spectrogram = rospy.Publisher('~spectrogram', Image)
        self.bridge = CvBridge()

    def _cb(self, msg):
        # save msg to audio_buffer
        data = msg.data
        audio_buffer = np.frombuffer(data, dtype=self.dtype)
        self.audio_buffer = np.append(
            self.audio_buffer, audio_buffer)
        self.audio_buffer = self.audio_buffer[
            -self.audio_buffer_len:]
        if len(self.audio_buffer) != self.audio_buffer_len:
            return
        # calc spectrogram by fft
        spectrum = np.fft.fft(self.audio_buffer * self.window_function)
        spectrum = np.log(np.abs(spectrum))
        spectrum = spectrum[self.cutoff_mask]
        # create spectrogram and publish
        self.spectrogram = np.concatenate(
            [self.spectrogram, spectrum[None]])
        self.spectrogram = self.spectrogram[1:].astype(np.float32)  # add new element to the queue
        spectrogram_pub = self.spectrogram.transpose(1, 0)[::-1, :]
        spectrogram_pub = self.bridge.cv2_to_imgmsg(spectrogram_pub, '32FC1')
        spectrogram_pub.header.stamp = rospy.Time.now()
        self.pub_spectrogram.publish(spectrogram_pub)


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrogram')
    AudioToSpectrogram()
    rospy.spin()
