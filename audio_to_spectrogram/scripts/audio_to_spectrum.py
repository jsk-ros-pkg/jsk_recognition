#!/usr/bin/env python

import numpy as np
import rospy

from audio_common_msgs.msg import AudioData
from jsk_recognition_msgs.msg import Spectrum


# This node execute FFT to audio (audio_common_msgs/AudioData)
# The format of audio topic is assumed to be wave.

class AudioToSpectrum(object):

    def __init__(self):
        super(AudioToSpectrum, self).__init__()

        # Audio topic config
        # The number of channels in audio data
        self.n_channel = rospy.get_param('~n_channel', 1)
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
        low_cut_freq = rospy.get_param('~low_cut_freq', 1)  # remove 0 Hz
        self.freq = np.fft.fftfreq(
            self.audio_buffer_len, d=1./mic_sampling_rate)
        self.cutoff_mask = np.where(
            (low_cut_freq <= self.freq) & (self.freq <= high_cut_freq),
            True, False)
        # How many times fft is executed in one second
        # fft_exec_rate equals to output spectrogram hz
        self.fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)

        # Publisher and Subscriber
        rospy.Subscriber(
            '~audio', AudioData, self.audio_cb)
        self.pub_spectrum = rospy.Publisher(
            '~spectrum', Spectrum, queue_size=1)
        self.pub_spectrum_filtered = rospy.Publisher(
            '~spectrum_filtered', Spectrum, queue_size=1)
        rospy.Timer(rospy.Duration(1. / self.fft_exec_rate), self.timer_cb)

    def audio_cb(self, msg):
        # Convert audio buffer to int array
        data = msg.data
        audio_buffer = np.frombuffer(data, dtype=self.dtype)
        # Retreive one channel data
        audio_buffer = audio_buffer[0::self.n_channel]
        # Save audio msg to audio_buffer
        self.audio_buffer = np.append(
            self.audio_buffer, audio_buffer)
        self.audio_buffer = self.audio_buffer[
            -self.audio_buffer_len:]

    def timer_cb(self, timer):
        if len(self.audio_buffer) != self.audio_buffer_len:
            return
        # Calc spectrum by fft
        amplitude = np.fft.fft(self.audio_buffer * self.window_function)
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
