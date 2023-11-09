#!/usr/bin/env python

import rospy

from audio_to_spectrogram import DataAmplitudePlot


if __name__ == '__main__':
    rospy.init_node('data_amplitude_plot')
    DataAmplitudePlot()
    rospy.spin()
