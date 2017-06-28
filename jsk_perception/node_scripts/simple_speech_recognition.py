#!/usr/bin/env python
# -*- coding: utf-8 -*-


try:
    import speech_recognition as sr
except:
    print "speech_recognition library is not installed. Try 'pip install SpeechRecognition'"
    exit(1)

import os
import pyaudio
import rospy
import subprocess
import tempfile
import traceback
import wave

from jsk_perception.srv import SpeechRecognition, SpeechRecognitionResponse
class CommandFailure(Exception):
    pass

def _cmd_exec(cmd, env=None):
    all_env = os.environ.copy()
    if env:
        all_env.update(env)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=all_env)
    stdout, stderr = p.communicate()
    if p.returncode != 0:
        raise CommandFailure("Command %s exits with non-zero code.\nError: %s" % (" ".join(cmd), stderr))
    return stdout

def recognize(duration=3.0, quiet=False,
              verbose=False, play_device=None, record_device=None,
              record_start_signal=None, record_end_signal=None,
              keywords=None,
              language="en-US", api="google", config={}):
    """Speech Recognition

    Args:
      duration (float): duration to record speech
      quiet (bool): By default, start and end of speech record are indicated by sound. If quiet is set True, recording speech audio starts and ends without any instruction.
      verbose (bool, default: False): Returns verbose result if true.
      keywords (list, default: None): Select candidates from given keyword phrases
      language (str, default: en-US): Language to recognize
      api (str, default: google): Service to use.
                                  (Available services: pocketsphinx, google, google_cloud, wit, bing, houndify or ibm)
      play_device (str, default: None): Device name to play
      record_device (str, default: None): Device name to record
      record_start_signal (str, default: None): audio file played when starts recording speech
      record_end_signal (str, default: None): audio file played when ends recording speech
      config (dict, default: {}): Credential keys
    Return:
      Recognized word (str) or None
    """

    if record_start_signal is None:
        record_start_signal = "/usr/share/sounds/ubuntu/stereo/bell.ogg"
    if record_end_signal is None:
        record_end_signal = "/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg"
    wave_file = os.path.join(tempfile._get_default_tempdir(),
                             next(tempfile._get_candidate_names()))

    rospy.loginfo("Recording voice: %s" % wave_file)

    if keywords:
        if api not in ["pocketsphinx", "google_cloud"]:
            rospy.logwarn("keyword phrases are enabled only with 'pocketsphinx' or 'google_cloud'")
        else:
            keywords = map(lambda x: (x, 0.0), keywords)

    play_cmd = ["play", "-q"]
    play_env = {}
    record_cmd = ["arecord", "-q"]
    if play_device:
        play_env = {"AUDIODEV": play_device}
    if record_device:
        record_cmd += ["-D", record_device]

    try:
        if not quiet:
            _cmd_exec(play_cmd + [record_start_signal], env=play_env)
        _cmd_exec(record_cmd + ["-f", "cd", "-d", str(duration), "-t", "wav", wave_file])
        if not quiet:
            _cmd_exec(play_cmd + [record_end_signal], env=play_env)

        r = sr.Recognizer()
        with sr.AudioFile(wave_file) as source:
            audio = r.listen(source)

        try:
            if api == "pocketsphinx":
                speech = r.recognize_sphinx(audio, language=language, keyword_entries=keywords, show_all=verbose)
            elif api == "google":
                speech = r.recognize_google(audio, language=language, show_all=verbose, **config)
            elif api == "google_cloud":
                speech = r.recognize_google_cloud(audio, language=language, preferred_phrases=keywords, show_all=verbose, **config)
            elif api == "wit":
                speech = r.recognize_wit(audio, show_all=verbose, **config)
            elif api == "bing":
                speech = r.recognize_bing(audio, language=language, show_all=verbose, **config)
            elif api == "houndify":
                speech = r.recognize_houndify(audio, show_all=verbose, **config)
            elif api == "ibm":
                speech = r.recognize_ibm(audio, language=language, show_all=verbose, **config)
            else:
                raise ValueError("Invalid Speech Recognition API: %s" % api)
            rospy.loginfo("Recognized: %s" % speech)
            return speech
        except sr.UnknownValueError:
            rospy.logerr("Recognition Service could not understand the audio")
        except sr.RequestError as e:
            rospy.logerr("Could not request results from Recognition Service: {0}".format(e))

    except Exception as e:
        rospy.logerr(traceback.format_exc())
        rospy.logerr(str(e))
        if os.path.exists(wave_file):
            os.remove(wave_file)

    return None


class SimpleSpeechRecognition(object):
    def __init__(self):

        self.api = rospy.get_param("~service", "google")
        self.language = rospy.get_param("~language", "en-US")
        self.unreliable = rospy.get_param("~unreliable", False)
        self.play_device = rospy.get_param("~play_device", None)
        self.record_device = rospy.get_param("~record_device", None)
        self.record_start_signal = rospy.get_param("~record_start_signal", None)
        self.record_end_signal = rospy.get_param("~record_end_signal", None)

        self.srv_sr = rospy.Service("speech_recognition",
                                    SpeechRecognition,
                                    self.speech_recognition_srv)

    def speech_recognition_srv(self, req):
        sentence = recognize(duration=req.duration,
                             quiet=req.quiet,
                             keywords=req.keywords,
                             verbose=self.unreliable,
                             language=self.language,
                             api=self.api,
                             play_device=self.play_device,
                             record_device=self.record_device,
                             record_start_signal=self.record_start_signal,
                             record_end_signal=self.record_end_signal,
        )
        res = SpeechRecognitionResponse()
        if sentence:
            res.sentence = sentence
        return res


if __name__ == '__main__':
    rospy.init_node("simple_speech_recognition")
    ssr = SimpleSpeechRecognition()
    rospy.spin()
