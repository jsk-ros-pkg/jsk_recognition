#!/usr/bin/env python

import numpy as np
import rospy

from audio_common_msgs.msg import AudioData
from jsk_recognition_msgs.msg import Spectrum

from audio_to_spectrogram import AudioBuffer

# This node execute FFT to audio (audio_common_msgs/AudioData)
# The format of audio topic is assumed to be wave.

class AudioToSpectrum(object):

    def __init__(self):
        super(AudioToSpectrum, self).__init__()

        self.audio_buffer = AudioBuffer.from_rosparam(auto_start=True)
        fft_sampling_period = rospy.get_param('~fft_sampling_period', 0.3)
        self.audio_buffer.window_size = fft_sampling_period
        mic_sampling_rate = self.audio_buffer.input_sample_rate

        # fft config
        window_function = np.arange(
            0.0, 1.0, 1.0 / self.audio_buffer.audio_buffer_len)
        self.window_function = 0.54 - 0.46 * np.cos(
            2 * np.pi * window_function)
        high_cut_freq = rospy.get_param('~high_cut_freq', 800)
        if high_cut_freq > mic_sampling_rate / 2:
            rospy.logerr('Set high_cut_freq lower than {} Hz'.format(
                mic_sampling_rate / 2))
        low_cut_freq = rospy.get_param('~low_cut_freq', 1)  # remove 0 Hz
        self.freq = np.fft.fftfreq(
            self.audio_buffer.audio_buffer_len, d=1./mic_sampling_rate)
        self.cutoff_mask = np.where(
            (low_cut_freq <= self.freq) & (self.freq <= high_cut_freq),
            True, False)
        # How many times fft is executed in one second
        # fft_exec_rate equals to output spectrogram hz
        self.fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)

        # Publisher and Subscriber
        self.pub_spectrum = rospy.Publisher(
            '~spectrum', Spectrum, queue_size=1)
        self.pub_spectrum_filtered = rospy.Publisher(
            '~spectrum_filtered', Spectrum, queue_size=1)
        rospy.Timer(rospy.Duration(1. / self.fft_exec_rate), self.timer_cb)

    def timer_cb(self, timer):
        if len(self.audio_buffer) != self.audio_buffer.audio_buffer_len:
            return
        audio_data = self.audio_buffer.read()
        # Calc spectrum by fft
        amplitude = np.fft.fft(audio_data * self.window_function)
        amplitude = np.log(np.abs(amplitude))
        spectrum_msg = Spectrum()
        spectrum_msg.header.stamp = rospy.Time.now()
        spectrum_msg.amplitude = amplitude
        spectrum_msg.frequency = self.freq
        self.pub_spectrum.publish(spectrum_msg)
        spectrum_msg.amplitude = amplitude[self.cutoff_mask]
        spectrum_msg.frequency = self.freq[self.cutoff_mask]
        self.pub_spectrum_filtered.publish(spectrum_msg)


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrum')
    AudioToSpectrum()
    rospy.spin()
