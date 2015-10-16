#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import math
import array
import audioop
import wave
import StringIO

import rospy
from audio_common_msgs.msg import AudioData
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from rospeex_if import ROSpeexInterface


class SpeechRecognitionNode(object):
    def __init__(self):
        self.language = rospy.get_param("~language", "ja")
        self.recog_engine = rospy.get_param("~recog_engine", "nict")
        self.sample_rate = rospy.get_param("~sample_rate", 16000)

        self.sif = ROSpeexInterface()
        self.sif.init(ss=False, spi=False, sr=True)
        self.sif.register_sr_response(self.recognition_cb)

        self.pub = rospy.Publisher("voice", SpeechRecognitionCandidates)
        self.sub = rospy.Subscriber("voice_phrase", AudioData, self.phrase_cb)

    def phrase_cb(self, msg):
        try:
            buf = array.array('h', msg.data).tostring()
        except Exception as e:
            rospy.logerr("received invalid data...: %s" % e)
            rospy.sleep(1)
            return

        f = StringIO.StringIO()
        wav = wave.open(f, 'wb')
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(self.sample_rate)
        wav.setcomptype("NONE", "not compressed")
        wav.writeframes(buf)
        wav.close()
        data = f.getvalue()
        f.close()

        self.sif.recognize(data=data,
                           language=self.language,
                           engine=self.recog_engine)

    def recognition_cb(self, msg):
        rospy.loginfo("heard: %s" % msg)
        pub_msg = SpeechRecognitionCandidates()
        pub_msg.transcript = [msg]
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node("speech_recognition_node")
    n = SpeechRecognitionNode()
    rospy.spin()
