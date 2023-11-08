#!/usr/bin/env python

from __future__ import division

import numpy as np
import rospy

from jsk_recognition_msgs.msg import Spectrum
from jsk_topic_tools import ConnectionBasedTransport


class SpectrumFilter(ConnectionBasedTransport):

    def __init__(self):
        super(SpectrumFilter, self).__init__()
        data_sampling_rate = rospy.get_param('~data_sampling_rate', 500)
        self.high_cut_freq = rospy.get_param('~high_cut_freq', 250)
        nyquist_freq = data_sampling_rate / 2.0
        if self.high_cut_freq > nyquist_freq:
            rospy.logerr('Set high_cut_freq not more than {} Hz'.format(
                nyquist_freq))
        self.low_cut_freq = rospy.get_param('~low_cut_freq', 0)

        self.pub = self.advertise('~output', Spectrum, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Spectrum, self.input_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def input_cb(self, sub_msg):
        amp = np.array(sub_msg.amplitude, dtype=np.float32)
        freq = np.array(sub_msg.frequency, dtype=np.float32)
        cutoff_mask = np.where(
            (self.low_cut_freq <= freq) & (freq <= self.high_cut_freq),
            True, False)
        pub_msg = Spectrum()
        pub_msg.header.stamp = sub_msg.header.stamp
        pub_msg.amplitude = amp[cutoff_mask]
        pub_msg.frequency = freq[cutoff_mask]
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('spectrum_filter')
    SpectrumFilter()
    rospy.spin()
