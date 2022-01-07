#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import threading

from audio_common_msgs.msg import AudioData
import cv_bridge
from dynamic_reconfigure.server import Server
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg

from audio_to_spectrogram.cfg import AudioAmplitudePlotConfig as Config
from audio_to_spectrogram import convert_matplotlib_to_img


class AudioAmplitudePlot(object):

    def __init__(self):
        super(AudioAmplitudePlot, self).__init__()

        self.lock = threading.Lock()

        # Audio topic config
        # The number of channels in audio data
        self.n_channel = rospy.get_param('~n_channel', 1)
        # Sampling rate of microphone (namely audio topic).
        self.mic_sampling_rate = rospy.get_param('~mic_sampling_rate', 16000)
        # Bits per one audio data
        bitdepth = rospy.get_param('~bitdepth', 16)
        if bitdepth == 16:
            self.dtype = 'int16'
        else:
            rospy.logerr("'~bitdepth' {} is unsupported.".format(bitdepth))

        # Set matplotlib config
        self.fig = plt.figure(figsize=(8, 5))
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_xlabel('Time [s]', fontsize=12)
        self.ax.set_ylabel('Amplitude', fontsize=12)
        self.line, = self.ax.plot([0, 0], label='Amplitude of Audio')

        # ROS dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        # ROS publisher and subscriber
        self.pub_img = rospy.Publisher(
            '~output/viz', sensor_msgs.msg.Image, queue_size=1)
        self.sub_audio = rospy.Subscriber(
            '~audio', AudioData, self.audio_cb)

        # set time callback.
        rate = rospy.get_param('~rate', 10)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 10.')
            rate = 10
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def config_callback(self, config, level):
        with self.lock:
            self.maximum_amplitude = config.maximum_amplitude
            self.window_size = config.window_size

            # Audio topic buffer
            self.audio_buffer = np.array([], dtype=self.dtype)
            # How long audio_buffer should be for one audio topic
            self.audio_buffer_len = int(
                self.mic_sampling_rate * self.window_size)

        return config

    def audio_cb(self, msg):
        # Convert audio buffer to int array
        data = msg.data
        audio_buffer = np.frombuffer(data, dtype=self.dtype)
        # Retreive one channel data
        audio_buffer = audio_buffer[0::self.n_channel]
        # Save audio msg to audio_buffer
        with self.lock:
            self.audio_buffer = np.append(
                self.audio_buffer, audio_buffer)
            self.audio_buffer = self.audio_buffer[
                -self.audio_buffer_len:]

    def timer_cb(self, timer):
        with self.lock:
            amp = self.audio_buffer.copy()
            times = np.linspace(-self.window_size, 0.0, len(amp))

        # Plot audio amplitude.
        self.line.set_data(times, amp)
        self.ax.set_xlim((times.min(), times.max()))
        self.ax.set_ylim((-self.maximum_amplitude, self.maximum_amplitude))

        self.ax.legend(loc='upper right')
        if self.pub_img.get_num_connections() > 0:
            bridge = cv_bridge.CvBridge()
            img = convert_matplotlib_to_img(self.fig)
            img_msg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
            img_msg.header.stamp = rospy.Time.now()
            self.pub_img.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('audio_amplitude_plot')
    AudioAmplitudePlot()
    rospy.spin()
