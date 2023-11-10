#!/usr/bin/env python

import rospy

from audio_to_spectrogram import AudioBuffer
from audio_to_spectrogram import DataAmplitudePlot


class AudioAmplitudePlot(DataAmplitudePlot):

    def __init__(self):
        # Overwrite dynamic_reconfigure's default values
        param_name = '~maximum_amplitude'
        if not rospy.has_param(param_name):
            rospy.set_param(param_name, 10000)

        super(AudioAmplitudePlot, self).__init__(
            data_buffer=AudioBuffer.from_rosparam(),
        )


if __name__ == '__main__':
    rospy.init_node('audio_amplitude_plot')
    AudioAmplitudePlot()
    rospy.spin()
