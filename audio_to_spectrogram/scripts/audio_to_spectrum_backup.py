#!/usr/bin/env python

import numpy as np
import rospy

from audio_common_msgs.msg import AudioData
from jsk_recognition_msgs.msg import Spectrum

import sys
import wavio
import os

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

        #self.mean_amplitude = np.empty(self.audio_buffer_len)
        self.mean_amplitude = 0
        self.first_100 = 0
        self.first = True
        self.mean_flag = False
        self.a_lis = np.array([])
        self.save_dir = os.path.dirname(os.path.abspath(__file__))
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
        #print("before:{}".format(self.audio_buffer * self.window_function))
        #print(self.audio_buffer)
        amplitude = np.fft.fft(self.audio_buffer * self.window_function)
        #c = np.array([1 if i>=0 else -1 for i in amplitude])
        phase = np.angle(amplitude)
        amplitude = np.log(np.abs(amplitude))
        
        if self.mean_flag:
            amplitude -= self.mean_amplitude
            #amplitude -= 0
            #print(amplitude.max())
            #self.mean_flag = False
        spectrum_msg = Spectrum()
        spectrum_msg.header.stamp = rospy.Time.now()
        spectrum_msg.amplitude = amplitude
        spectrum_msg.frequency = self.freq
        self.pub_spectrum.publish(spectrum_msg)
        spectrum_msg.amplitude = amplitude[self.cutoff_mask]
        spectrum_msg.frequency = self.freq[self.cutoff_mask]
        self.pub_spectrum_filtered.publish(spectrum_msg)

        if self.first_100 < 200:
            #np.set_printoptions(threshold=1000000)
            #print(amplitude)
            self.mean_amplitude += amplitude
            #print(self.mean_amplitude)
            #print(np.count_nonzero(np.isnan(self.mean_amplitude)))
            #sys.exit()
            #self.a_lis = np.append(self.a_lis, self.audio_buffer)
        else:
            if self.first:
                self.first = False
                self.mean_amplitude /= 200
                print("ok")
                self.mean_flag = True
                #print(self.a_lis)
                #wavio.write(os.path.join(self.save_dir + "out.wav"), self.a_lis, 16000, sampwidth=3)
        self.first_100 += 1
        #print(amplitude.shape) #4800
        
        #print(amplitude)
        reverse = np.exp(amplitude)
        reverse = np.array(reverse * np.cos(phase) + (reverse * np.sin(phase))*1.j)
        #print(reverse[0:10])
        reverse = np.fft.ifft(reverse)
        reverse = reverse.real
        #print("after:{}".format(reverse))
        #print(reverse)
        reverse = reverse / self.window_function
        #print(reverse)
        #print(reverse.max())

if __name__ == '__main__':
    rospy.init_node('audio_to_spectrum')
    AudioToSpectrum()
    rospy.spin()
