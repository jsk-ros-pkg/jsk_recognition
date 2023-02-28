#!/usr/bin/env python

import rospy

from audio_to_spectrogram import AudioBuffer
from audio_to_spectrogram import DataToSpectrum

# This node execute FFT to audio (audio_common_msgs/AudioData)
# The format of audio topic is assumed to be wave.

class AudioToSpectrum(DataToSpectrum):

    def __init__(self):
        super(AudioToSpectrum, self).__init__(
            data_buffer=AudioBuffer.from_rosparam(auto_start=True),
            high_cut_freq=rospy.get_param('~high_cut_freq', 800),
            low_cut_freq=rospy.get_param('~low_cut_freq', 1),  # remove 0 Hz
        )


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrum')
    AudioToSpectrum()
    rospy.spin()
