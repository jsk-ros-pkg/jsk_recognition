#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import math
import cv2
from cv_bridge import CvBridge
from distutils.version import LooseVersion
from dynamic_reconfigure.server import Server
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib import gridspec
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ColorHistogram, ColorHistogramArray
from sensor_msgs.msg import Image
from jsk_pcl_ros.cfg import ColorHistogramVisualizerConfig as Config


# matplotlib outputs image with size 800x600 by default
IMG_WIDTH=800
IMG_HEIGHT=600


class ColorHistogramVisualizer(ConnectionBasedTransport):
    def __init__(self):
        super(ColorHistogramVisualizer, self).__init__()

        self.dyn_server = Server(Config, self.config_callback)

        self.cv_bridge = CvBridge()
        self.hsv_color_map = plt.cm.get_cmap('hsv')
        self.hsv_color_map_2d = self.get_hsv_color_map_2d()

        self.pub_image = self.advertise("~output", Image, queue_size=1)

    def config_callback(self, config, level):
        self.histogram_policy = config.histogram_policy
        self.histogram_index = config.histogram_index
        self.histogram_scale = config.histogram_scale
        return config

    def subscribe(self):
        self.sub_histogram = rospy.Subscriber("~input", ColorHistogram,
                                              self.callback, queue_size=1)
        self.sub_histogram_array = rospy.Subscriber("~input/array", ColorHistogramArray,
                                                    self.callback_array, queue_size=1)

    def unsubscribe(self):
        self.sub_histogram.unregister()
        self.sub_histogram_array.unregister()

    def get_hsv_color_map_2d(self):
        hsv_map = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8)
        h, s = np.indices(hsv_map.shape[:2])
        hsv_map[:, :, 0] = h / float(IMG_HEIGHT) * 180.0
        hsv_map[:, :, 1] = 125.0
        hsv_map[:, :, 2] = s / float(IMG_WIDTH)  * 256.0
        hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HLS2BGR)
        return hsv_map

    def callback(self, msg):
        if self.histogram_policy == Config.ColorHistogramVisualizer_HUE:
            img = self.plot_hist_hue(msg.histogram)
        elif self.histogram_policy == Config.ColorHistogramVisualizer_HUE_AND_SATURATION:
            img = self.plot_hist_hs(msg.histogram)
        else:
            rospy.logerr("Invalid histogram policy")
            return
        try:
            pub_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
            self.pub_image.publish(pub_msg)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s" % str(e))

    def callback_array(self, msg):
        if 0 <= self.histogram_index < len(msg.histograms):
            self.callback(msg.histograms[self.histogram_index])
        else:
            rospy.logerr("histogram index Out-of-index")

    def image_from_plot(self):
        fig = plt.gcf()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
        fig.clf()
        img.shape = (h, w, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def plot_hist_hue(self, hist):
        gs = gridspec.GridSpec(1, 2, width_ratios=[4, 1])
        if LooseVersion(matplotlib.__version__).version[0] > 1:
            plt.subplot(gs[0], facecolor='silver')
        else:  # matplotlib version < 2.0.0
            plt.subplot(gs[0], axisbg='silver')
        bin_size = len(hist) - 2
        bin_step = 360.0 / bin_size
        x = np.arange(360.0, step=bin_step)
        bars = plt.bar(x, hist[:-2], width=bin_step)
        cs = np.arange(0.0, 1.0, 1.0 / bin_size)
        for c, b in zip(cs, bars):
            b.set_facecolor(self.hsv_color_map(c))
        plt.xlim(0, 360.0)
        plt.ylim(ymin=0.0, ymax=1.0)
        ymin, ymax = plt.ylim()
        if LooseVersion(matplotlib.__version__).version[0] > 1:
            plt.subplot(gs[1], facecolor='silver')
        else:  # matplotlib version < 2.0.0
            plt.subplot(gs[1], axisbg='silver')
        bars = plt.bar(range(2), hist[-2:], label=["white", "black"],
                       width=1.0, linewidth=2.0)
        bars[0].set_facecolor((1.0, 1.0, 1.0, 1.0))
        bars[1].set_facecolor((0.0, 0.0, 0.0, 1.0))
        plt.ylim(ymin, ymax)
        return self.image_from_plot()

    def plot_hist_saturation(self, hist):
        bin_size = len(hist)
        bin_step = 256.0 / bin_size
        x = np.arange(256.0, step=bin_step)
        plt.bar(x, hist, width=bin_step)
        plt.xlim(0, 256.0)
        return self.image_from_plot()

    def plot_hist_hs(self, hist):
        bin_size = int(math.sqrt(len(hist) - 2))
        white, black = hist[-2], hist[-1]
        hist = np.array(hist[:-2]).reshape(bin_size, bin_size).T
        rospy.loginfo("white: %f, black: %f" % (white, black))
        hist = np.clip(hist * 150 * self.histogram_scale, white, 1.0 - black)
        hist = hist[:, :, np.newaxis]
        hist = cv2.resize(hist, (IMG_WIDTH, IMG_HEIGHT))
        hist = hist[:, :, np.newaxis]
        img = self.hsv_color_map_2d * hist
        img = img.astype(np.uint8)
        return img

if __name__ == '__main__':
    rospy.init_node("color_histogram_visualizer")
    v = ColorHistogramVisualizer()
    rospy.spin()
