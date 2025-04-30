#!/usr/bin/env python

import rospy

from audio_to_spectrogram import DataToSpectrum

# This node execute FFT to input data

if __name__ == '__main__':
    rospy.init_node('data_to_spectrum')
    DataToSpectrum()
    rospy.spin()
