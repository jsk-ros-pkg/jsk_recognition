#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_topic_tools import ConnectionBasedTransport
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg

from audio_to_spectrogram import AudioBuffer
from audio_to_spectrogram.cfg import AudioAmplitudePlotConfig as Config
from audio_to_spectrogram import convert_matplotlib_to_img


class AudioAmplitudePlot(ConnectionBasedTransport):

    def __init__(self):
        super(AudioAmplitudePlot, self).__init__()

        self.audio_buffer = AudioBuffer.from_rosparam()

        # Set matplotlib config
        self.fig = plt.figure(figsize=(8, 5))
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_xlabel('Time [s]', fontsize=12)
        self.ax.set_ylabel('Amplitude', fontsize=12)
        self.line, = self.ax.plot([0, 0], label='Amplitude of Audio')

        # ROS dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        self.pub_img = self.advertise(
            '~output/viz', sensor_msgs.msg.Image, queue_size=1)

    def start_timer(self):
        rate = rospy.get_param('~rate', 10)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 10.')
            rate = 10
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def stop_timer(self):
        self.timer.shutdown()

    def subscribe(self):
        self.audio_buffer.subscribe()
        self.start_timer()

    def unsubscribe(self):
        self.audio_buffer.unsubscribe()
        self.stop_timer()

    def config_callback(self, config, level):
        self.maximum_amplitude = config.maximum_amplitude
        self.window_size = config.window_size
        self.audio_buffer.window_size = self.window_size
        return config

    def timer_cb(self, timer):
        window_size = self.window_size
        amp = self.audio_buffer.read(window_size)
        if len(amp) == 0:
            return
        times = np.linspace(-window_size, 0.0, len(amp))

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
