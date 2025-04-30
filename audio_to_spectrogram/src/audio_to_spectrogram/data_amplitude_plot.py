from __future__ import division

from distutils.version import LooseVersion
import pkg_resources
import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_topic_tools import ConnectionBasedTransport
import matplotlib
from audio_to_spectrogram.compat import check_matplotlib_version; check_matplotlib_version()
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sensor_msgs.msg

from audio_to_spectrogram.cfg import DataAmplitudePlotConfig as Config
from audio_to_spectrogram.convert_matplotlib import convert_matplotlib_to_img
from audio_to_spectrogram.data_buffer import DataBuffer


class DataAmplitudePlot(ConnectionBasedTransport):

    def __init__(self, data_buffer=None):
        super(DataAmplitudePlot, self).__init__()

        if data_buffer is None:
            self.data_buffer = DataBuffer.from_rosparam()
        else:
            self.data_buffer = data_buffer

        # Set matplotlib config
        self.fig = plt.figure(figsize=(8, 5))
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_xlabel('Time [s]', fontsize=12)
        self.ax.set_ylabel('Amplitude', fontsize=12)
        self.line, = self.ax.plot([0, 0], label='Amplitude of Input Data')

        # ROS dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        self.pub_img = self.advertise(
            '~output/viz', sensor_msgs.msg.Image, queue_size=1)

    def start_timer(self):
        rate = rospy.get_param('~rate', 10)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 10.')
            rate = 10

        timer_kwargs = dict(
            period=rospy.Duration(1.0 / rate),
            callback=self.timer_cb,
            oneshot=False,
        )
        if (LooseVersion(pkg_resources.get_distribution('rospy').version) >=
                LooseVersion('1.12.0')) and rospy.get_param('/use_sim_time', None):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            timer_kwargs['reset'] = True
        self.timer = rospy.Timer(**timer_kwargs)

    def stop_timer(self):
        self.timer.shutdown()
        self.timer = None

    def subscribe(self):
        self.data_buffer.subscribe()
        self.start_timer()

    def unsubscribe(self):
        self.data_buffer.unsubscribe()
        self.stop_timer()

    def config_callback(self, config, level):
        self.maximum_amplitude = config.maximum_amplitude
        self.window_size = config.window_size
        self.data_buffer.window_size = self.window_size
        return config

    def timer_cb(self, timer):
        window_size = self.window_size
        amp = self.data_buffer.read(window_size)
        if len(amp) == 0:
            return
        times = np.linspace(-window_size, 0.0, len(amp))

        # Plot data amplitude.
        self.line.set_data(times, amp)
        self.ax.set_xlim((times.min(), times.max()))
        self.ax.set_ylim((-self.maximum_amplitude, self.maximum_amplitude))

        self.ax.legend(loc='upper right')
        self.fig.tight_layout()
        if self.pub_img.get_num_connections() > 0:
            bridge = cv_bridge.CvBridge()
            img = convert_matplotlib_to_img(self.fig)
            img_msg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
            img_msg.header.stamp = rospy.Time.now()
            self.pub_img.publish(img_msg)
