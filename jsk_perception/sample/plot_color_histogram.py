#!/usr/bin/env python

"""Node for visualization in sample program.

This will be replaced from rqt_plot_hist.
https://github.com/ros-visualization/rqt_common_plugins/pull/442
"""

import matplotlib
matplotlib.use('Agg')  # NOQA
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image

import cv_bridge
from jsk_recognition_msgs.msg import ColorHistogram
from jsk_topic_tools import ConnectionBasedTransport
import rospy


class PlotHistogram(ConnectionBasedTransport):

    def __init__(self):
        super(PlotHistogram, self).__init__()
        self.pub = self.advertise('~output', Image, queue_size=1)
        self.fig = plt.figure()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', ColorHistogram, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        ax = self.fig.add_subplot(111)
        ax.bar(np.arange(len(msg.histogram)), msg.histogram)

        self.fig.canvas.draw()

        width, height = self.fig.canvas.get_width_height()
        img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(height, width, 3)

        plt.cla()

        bridge = cv_bridge.CvBridge()
        imgmsg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
        imgmsg.header = msg.header
        self.pub.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('plot_histogram')
    PlotHistogram()
    rospy.spin()
