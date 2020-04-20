#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import matplotlib.pyplot as plt
import numpy as np
import rospy

from jsk_recognition_msgs.msg import Spectrum


class SpectrumPlot():

    def __init__(self):
        # Set matplotlib config
        self.fig = plt.figure(figsize=(8, 5))
        self.fig.suptitle('Spectrum plot', size=12)
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.90, bottom=0.1,
                                 wspace=0.2, hspace=0.6)
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_xlabel('Frequency [Hz]', fontsize=12)
        self.ax.set_ylabel('Amplitude', fontsize=12)
        self.line, = self.ax.plot([0, 0], label='Amplitude of Spectrum')
        # ROS subscriber
        self.sub_spectrum = rospy.Subscriber(
            '~spectrum', Spectrum, self._cb, queue_size=1000)

    def _cb(self, msg):
        # Plot spectrum
        self.amp = np.array(msg.amplitude)
        self.freq = np.array(msg.frequency)
        self.line.set_data(self.freq, self.amp)
        self.ax.set_xlim((self.freq.min(), self.freq.max()))
        self.ax.set_ylim((0.0, 20))
        self.ax.legend(loc='upper right')


if __name__ == '__main__':
    rospy.init_node('spectrum_plot')
    SpectrumPlot()
    while not rospy.is_shutdown():
        plt.pause(.1)  # real-time plotting
