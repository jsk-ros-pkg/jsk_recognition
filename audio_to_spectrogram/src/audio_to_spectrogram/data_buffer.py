from __future__ import division

import array
from threading import Lock

import numpy as np
import rospy
import rostopic


class DataBuffer(object):

    def __init__(self, topic_name='~input',
                 expr_to_get_data='m.data',
                 input_sample_rate=500,
                 window_size=10.0,
                 is_integer=False,
                 is_signed=True,
                 bitdepth=64,
                 n_channel=1, target_channel=0,
                 get_latest_data=False,
                 discard_data=False,
                 auto_start=False):
        self.is_subscribing = True
        self.get_latest_data = get_latest_data
        self.discard_data = discard_data
        self._window_size = window_size
        self.data_buffer_len = int(self._window_size * input_sample_rate)
        self.lock = Lock()
        self.bitdepth = bitdepth
        self.n_channel = n_channel
        self.target_channel = min(self.n_channel - 1, max(0, target_channel))
        self.input_sample_rate = input_sample_rate
        self.type_code = {}
        if is_integer:
            if is_signed:
                codes = ['b', 'h', 'i', 'l']
            else:
                codes = ['B', 'H', 'I', 'L']
        else:
            codes = ['f', 'd']
        for code in codes:
            self.type_code[array.array(code).itemsize] = code

        self.dtype = self.type_code[self.bitdepth / 8]
        self.data_buffer = np.array([], dtype=self.dtype)

        if is_integer:
            self.max_value = 2 ** (self.bitdepth - 1) - 1
        else:
            self.max_value = None

        self.topic_name = topic_name
        self.expr_to_get_data = expr_to_get_data

        if auto_start:
            self.subscribe()

    def __len__(self):
        return len(self.data_buffer)

    @property
    def window_size(self):
        return self._window_size

    @window_size.setter
    def window_size(self, size):
        with self.lock:
            self._window_size = size
            self.data_buffer_len = int(self._window_size
                                       * self.input_sample_rate)
            self.data_buffer = np.array([], dtype=self.dtype)

    @staticmethod
    def from_rosparam(**kwargs):
        expr_to_get_data = rospy.get_param(
            '~expression_to_get_data', 'm.data')
        n_channel = rospy.get_param('~n_channel', 1)
        target_channel = rospy.get_param('~target_channel', 0)
        data_sampling_rate = rospy.get_param('~data_sampling_rate', 500)
        is_integer = rospy.get_param('~is_integer', False)
        is_signed = rospy.get_param('~is_signed', True)
        bitdepth = rospy.get_param('~bitdepth', 64)
        return DataBuffer(expr_to_get_data=expr_to_get_data,
                          input_sample_rate=data_sampling_rate,
                          is_integer=is_integer,
                          is_signed=is_signed,
                          bitdepth=bitdepth,
                          n_channel=n_channel,
                          target_channel=target_channel,
                          **kwargs)

    def subscribe(self):
        self.data_buffer = np.array([], dtype=self.dtype)
        # https://github.com/ros/ros_comm/blob/1.14.13/tools/topic_tools/scripts/transform#L86-L87
        input_class, input_topic, self.input_fn = rostopic.get_topic_class(
            rospy.resolve_name(self.topic_name), blocking=True)
        # https://github.com/ros/ros_comm/blob/1.14.13/tools/topic_tools/scripts/transform#L95
        self.sub = rospy.Subscriber(
            input_topic, input_class, self.input_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _read(self, size, normalize=False):
        with self.lock:
            if self.get_latest_data:
                data_buffer = self.data_buffer[-size:]
            else:
                data_buffer = self.data_buffer[:size]
                if self.discard_data:
                    self.data_buffer = self.data_buffer[size:]
        if normalize is True:
            if self.max_value is not None:
                data_buffer = data_buffer / self.max_value
            else:
                rospy.logerr('normalize is disabled with current settings')
        return data_buffer

    def sufficient_data(self, size):
        return len(self.data_buffer) < size

    def read(self, size=None, wait=False, normalize=False):
        if size is None:
            size = self.data_buffer_len
        size = int(size * self.input_sample_rate)
        while wait is True \
                and not rospy.is_shutdown() and len(self.data_buffer) < size:
            rospy.sleep(0.001)
        return self._read(size, normalize=normalize)

    def close(self):
        try:
            self.sub.unregister()
        except Exception:
            pass
        self.data_buffer = np.array([], dtype=self.dtype)

    def input_cb(self, m):
        # https://github.com/ros/ros_comm/blob/1.14.13/tools/topic_tools/scripts/transform#L98-L99
        if self.input_fn is not None:
            m = self.input_fn(m)
        data = eval(self.expr_to_get_data)
        data_type = type(data)
        if data_type is bytes:
            data_buffer = np.frombuffer(data, dtype=self.dtype)
        elif data_type in [list, tuple]:
            data_buffer = np.fromiter(data, dtype=self.dtype)
        elif data_type in [int, float]:
            data_buffer = np.array([data], dtype=self.dtype)
        else:
            rospy.logerr('Unsupported data type: {}'.format(data_type))
            return
        # This division into cases assumes
        # expr_to_get_data returns a Python built-in numeric or sequence type
        # deserialized from a ROS msg by scripts generated by genpy.
        # Other than uint8[], this deserialization is executed with
        # struct.Struct.unpack() and it returns int or float (or its tuple).
        # Against uint8[], this deserialization returns bytes.
        # /opt/ros/melodic/lib/python2.7/dist-packages/std_msgs/msg/_Float64.py
        # /opt/ros/melodic/lib/python2.7/dist-packages/audio_common_msgs/msg/_AudioData.py
        # https://docs.python.org/2.7/library/struct.html#format-characters
        # https://github.com/ros/genpy/blob/0.6.16/src/genpy/generate_struct.py#L165

        data_buffer = data_buffer[self.target_channel::self.n_channel]
        with self.lock:
            self.data_buffer = np.append(
                self.data_buffer, data_buffer)
            self.data_buffer = self.data_buffer[
                -self.data_buffer_len:]
