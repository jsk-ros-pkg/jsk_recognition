from __future__ import division

import numpy as np
import rospy
from distutils.version import LooseVersion
import pkg_resources

from jsk_recognition_msgs.msg import Spectrum

from audio_to_spectrogram.data_buffer import DataBuffer


class DataToSpectrum(object):

    def __init__(self, data_buffer=None):
        super(DataToSpectrum, self).__init__()

        if data_buffer is None:
            self.data_buffer = DataBuffer.from_rosparam(auto_start=True)
        else:
            self.data_buffer = data_buffer
        fft_sampling_period = rospy.get_param('~fft_sampling_period', 0.3)
        self.data_buffer.window_size = fft_sampling_period
        data_sampling_rate = self.data_buffer.input_sample_rate

        # fft config
        window_function = np.arange(
            0.0, 1.0, 1.0 / self.data_buffer.data_buffer_len)
        self.window_function = 0.54 - 0.46 * np.cos(
            2 * np.pi * window_function)
        self.freq = np.fft.fftfreq(
            self.data_buffer.data_buffer_len, d=1./data_sampling_rate)
        # How many times fft is executed in one second
        # fft_exec_rate equals to output spectrogram hz
        self.fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)

        # Publisher and Subscriber
        self.pub_spectrum = rospy.Publisher(
            '~spectrum', Spectrum, queue_size=1)
        self.pub_log_spectrum = rospy.Publisher(
            '~log_spectrum', Spectrum, queue_size=1)

        timer_kwargs = dict(
            period=rospy.Duration(1.0 / self.fft_exec_rate),
            callback=self.timer_cb,
            oneshot=False,
        )
        if (LooseVersion(pkg_resources.get_distribution('rospy').version) >=
                LooseVersion('1.12.0')) and rospy.get_param('/use_sim_time', None):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            timer_kwargs['reset'] = True
        self.timer = rospy.Timer(**timer_kwargs)

    def publish_spectrum(self, pub, amplitude):
        spectrum_msg = Spectrum()
        spectrum_msg.header.stamp = rospy.Time.now()
        spectrum_msg.amplitude = amplitude
        spectrum_msg.frequency = self.freq
        pub.publish(spectrum_msg)

    def timer_cb(self, timer):
        if len(self.data_buffer) != self.data_buffer.data_buffer_len:
            return
        data = self.data_buffer.read()
        # Calc spectrum by fft
        fft = np.fft.fft(data * self.window_function)
        self.publish_spectrum(
            self.pub_spectrum,
            np.abs(fft / (self.data_buffer.data_buffer_len / 2)),
            # Usual "amplitude spectrum" is np.abs(fft),
            # but we use the above equation to get "amplitude"
            # consistent with the amplitude of the original signal.
            # https://github.com/jsk-ros-pkg/jsk_recognition/issues/2761#issue-1550715400
        )
        self.publish_spectrum(
            self.pub_log_spectrum,
            np.log(np.abs(fft)),
            # Usually, log is applied to "power spectrum" (np.abs(fft)**2):
            # np.log(np.abs(fft)**2), 20*np.log10(np.abs(fft)), ...
            # But we use the above equation for simplicity.
            # https://github.com/jsk-ros-pkg/jsk_recognition/issues/2761#issuecomment-1445810380
            # http://makotomurakami.com/blog/2020/05/23/5266/
        )
