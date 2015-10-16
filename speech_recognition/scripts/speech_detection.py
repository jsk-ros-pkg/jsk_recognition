#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import math
import array
import audioop

import rospy
from audio_common_msgs.msg import AudioData


class SpeechDetectionNode(object):
    def __init__(self):
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.depth = rospy.get_param("~depth", 16)

        self.threshold_adjusted = not rospy.get_param("~auto_adjust_threshold", True)
        self.adjust_duration = rospy.get_param("~auto_adjust_duration", 1.0)
        self.buffer_for_adjust = ""
        self.elapsed_time_for_adjust = 0
        self.energy_adjust_damping = 0.15
        self.energy_adjust_ratio = 1.5
        self.energy_threshold = rospy.get_param("~energy_threshold", 300)

        self.speaking = False
        self.buffer_for_phrase = ""
        self.phrase_end_duration = rospy.get_param("~phrase_end_duration", 0.8)
        self.phrase_end_count = 0

        self.pub = rospy.Publisher("voice_phrase", AudioData)
        self.sub = rospy.Subscriber("voice_raw", AudioData, self.voice_raw_cb)

    def adjust_for_ambient_noise(self, buf):
        sample_width = self.depth / 8
        frames_per_buffer = len(buf) * 1.0 / sample_width
        seconds_per_buffer = frames_per_buffer / self.sample_rate
        if self.elapsed_time_for_adjust < self.adjust_duration:
            rospy.loginfo("adjusting... %f%%" % (100.0 * self.elapsed_time_for_adjust / self.adjust_duration))
            self.buffer_for_adjust += buf
            self.elapsed_time_for_adjust += seconds_per_buffer
        else:
            energy = audioop.rms(self.buffer_for_adjust, sample_width)
            damping = self.energy_adjust_damping ** seconds_per_buffer
            target_energy = energy * self.energy_adjust_ratio
            self.energy_threshold = self.energy_threshold * damping + target_energy * (1.0 - damping)
            self.threshold_adjusted = True
            self.buffer_for_adjust = ""
            self.elapsed_time_for_adjust = 0
            rospy.loginfo("energy_threshold is automatically adjusted to: %s" % self.energy_threshold)

    def voice_raw_cb(self, msg):
        try:
            buf = array.array('h', msg.data).tostring()
        except: return

        self.buffer_for_phrase += buf

        sample_width = self.depth / 8
        frames_per_buffer = len(buf) * 1.0 / sample_width
        seconds_per_buffer = frames_per_buffer / self.sample_rate
        phrase_end_buffer_count = int(math.ceil(self.phrase_end_duration / seconds_per_buffer))

        rospy.logdebug("frames_per_buffer: %f" % frames_per_buffer)
        rospy.logdebug("seconds_per_buffer: %f" % seconds_per_buffer)
        rospy.logdebug("phrase_end_buffer_count: %d" % phrase_end_buffer_count)

        energy = audioop.rms(buf, sample_width)

        rospy.logdebug("energy: %s" % energy)

        if not self.threshold_adjusted:
            self.adjust_for_ambient_noise(buf)
            return

        if energy > self.energy_threshold:
            if not self.speaking:
                self.speaking = True
                self.phrase_end_count = 0
                self.buffer_for_phrase = ""
                rospy.loginfo("start of phrase")
        else:
            if self.speaking:
                self.phrase_end_count += 1
                if self.phrase_end_count > phrase_end_buffer_count:
                    self.speaking = False
                    self.pub.publish(AudioData(data=self.buffer_for_phrase))
                    rospy.loginfo("end of phrase")

if __name__ == '__main__':
    rospy.init_node("speech_detection_node")
    n = SpeechDetectionNode()
    rospy.spin()
