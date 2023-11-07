#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import matplotlib
from audio_to_spectrogram import check_matplotlib_version; check_matplotlib_version()
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg

from jsk_recognition_msgs.msg import Spectrum

from audio_to_spectrogram import convert_matplotlib_to_img


class SpectrumPlot(ConnectionBasedTransport):

    def __init__(self):
        super(SpectrumPlot, self).__init__()
        self.min_amp = rospy.get_param('~min_amp', 0.0)
        self.max_amp = rospy.get_param('~max_amp', 20.0)
        self.queue_size = rospy.get_param('~queue_size', 1)
        max_rate = rospy.get_param('~max_rate', -1)
        if max_rate > 0:
            self.min_period = 1.0 / max_rate
        else:
            self.min_period = -1
        self.prev_pub_tm = None
        # Set matplotlib config
        self.fig = plt.figure(figsize=(8, 5))
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.90, bottom=0.1,
                                 wspace=0.2, hspace=0.6)
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.grid(True)
        # self.fig.suptitle('Spectrum plot', size=12)
        self.ax.set_title('Spectrum plot', fontsize=12)
        # Use self.ax.set_title() instead of
        # self.fig.suptitle() to use self.fig.tight_layout()
        # preventing characters from being cut off
        # cf. https://tm23forest.com/contents/matplotlib-tightlayout-with-figure-suptitle
        #     https://matplotlib.org/2.2.4/tutorials/intermediate/tight_layout_guide.html
        self.ax.set_xlabel('Frequency [Hz]', fontsize=12)
        self.ax.set_ylabel('Amplitude', fontsize=12)
        self.line, = self.ax.plot([0, 0], label='Amplitude of Spectrum')
        # ROS publisher and subscriber
        self.pub_img = self.advertise(
            '~output/viz', sensor_msgs.msg.Image, queue_size=1)

    def subscribe(self):
        self.sub_spectrum = rospy.Subscriber(
            '~spectrum', Spectrum, self._cb, queue_size=self.queue_size)

    def unsubscribe(self):
        self.sub_spectrum.unregister()

    def _cb(self, msg):
        # Keep max_rate
        if self.min_period > 0:
            curr_tm = rospy.Time.now().to_sec()
            if self.prev_pub_tm is not None:
                elapsed_tm = curr_tm - self.prev_pub_tm
                if elapsed_tm < 0:
                    # Time moved backwards (e.g., rosbag play --loop)
                    pass
                elif elapsed_tm < self.min_period:
                    return
            self.prev_pub_tm = curr_tm
        # Plot spectrum
        self.amp = np.array(msg.amplitude)
        self.freq = np.array(msg.frequency)
        self.line.set_data(self.freq, self.amp)
        self.ax.set_xlim((self.freq.min(), self.freq.max()))
        self.ax.set_ylim((self.min_amp, self.max_amp))
        self.ax.legend(loc='upper right')
        self.fig.tight_layout()
        if self.pub_img.get_num_connections() > 0:
            bridge = cv_bridge.CvBridge()
            img = convert_matplotlib_to_img(self.fig)
            img_msg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
            img_msg.header = msg.header
            self.pub_img.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('spectrum_plot')
    SpectrumPlot()
    rospy.spin()
