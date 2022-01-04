import array
from threading import Lock

from audio_common_msgs.msg import AudioData
import librosa
import numpy as np
import rospy
import std_msgs.msg


class AudioStream(object):

    def __init__(self, topic_name='~audio',
                 input_sample_rate=16000, output_sample_rate=None,
                 buffer_size=None,
                 depth=16,
                 n_channel=1, target_channel=0,
                 get_latest_data=False,
                 discard_data=True,
                 is_speeching_topic_name=None):
        self.is_subscribing = True
        self.get_latest_data = get_latest_data
        self.discard_data = discard_data
        self.audio_buffer_len = buffer_size or input_sample_rate * 5
        self.lock = Lock()
        self.depth = depth
        self.n_channel = n_channel
        self.target_channel = min(self.n_channel - 1, max(0, target_channel))
        self.input_sample_rate = input_sample_rate
        self.output_sample_rate = output_sample_rate or self.input_sample_rate
        self.type_code = {}
        self.is_speeching_topic_name = is_speeching_topic_name
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code

        self.dtype = self.type_code[self.depth / 8]
        self.audio_buffer = np.array([], dtype=self.dtype)

        self.max_value = 2 ** (self.depth - 1) - 1

        self.topic_name = topic_name
        self.sub_audio = rospy.Subscriber(
            self.topic_name, AudioData, self.audio_cb)
        if self.is_speeching_topic_name is not None:
            self.sub_speeching = rospy.Subscriber(
                self.is_speeching_topic_name,
                std_msgs.msg.Bool,
                self.is_speaking_topic_callback,
                queue_size=1)

    def is_speaking_topic_callback(self, msg):
        self.is_speeching = msg.data
        if self.is_subscribing is True and self.is_speeching is True:
            with self.lock:
                self.is_subscribing = False
                self.sub_audio.unregister()
        elif self.is_subscribing is False and self.is_speeching is False:
            with self.lock:
                self.is_subscribing = True
                self.audio_buffer = np.array([], dtype=self.dtype)
                self.sub_audio = rospy.Subscriber(
                    self.topic_name, AudioData, self.audio_cb)

    def _read(self, size, normalize=False):
        with self.lock:
            if self.get_latest_data:
                audio_buffer = self.audio_buffer[-size:]
            else:
                audio_buffer = self.audio_buffer[:size]
                if self.discard_data:
                    self.audio_buffer = self.audio_buffer[size:]
        if self.input_sample_rate != self.output_sample_rate:

            audio_buffer = librosa.resample(
                audio_buffer / self.max_value,
                orig_sr=self.input_sample_rate,
                target_sr=self.output_sample_rate)
            audio_buffer = np.array(
                np.clip(-self.max_value, self.max_value,
                        audio_buffer * self.max_value),
                dtype=self.dtype)
        if normalize is True:
            audio_buffer = audio_buffer / self.max_value
        return audio_buffer

    def sufficient_data(self, size):
        return len(self.audio_buffer) < size

    def read(self, size, normalize=False):
        if self.input_sample_rate and self.output_sample_rate:
            size = int(size * (self.input_sample_rate
                               / self.output_sample_rate))
        while not rospy.is_shutdown() and len(self.audio_buffer) < size:
            rospy.sleep(0.001)
        return self._read(size, normalize=normalize)

    def close(self):
        try:
            self.sub_audio.unregister()
        except Exception:
            pass
        self.audio_buffer = np.array([], dtype=self.dtype)

    def audio_cb(self, msg):
        audio_buffer = np.frombuffer(msg.data, dtype=self.dtype)
        audio_buffer = audio_buffer[self.target_channel::self.n_channel]
        with self.lock:
            self.audio_buffer = np.append(
                self.audio_buffer, audio_buffer)
            self.audio_buffer = self.audio_buffer[
                -self.audio_buffer_len:]
