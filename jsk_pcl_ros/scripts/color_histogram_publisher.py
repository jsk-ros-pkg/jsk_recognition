#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from jsk_recognition_msgs.msg import ColorHistogram

class ColorHistogramPublisher():

    def __init__(self):
        self.histogram = rospy.get_param('~histogram', [])
        self.frame_id = rospy.get_param('~frame_id')
        self.pub = rospy.Publisher('~output', ColorHistogram, queue_size=1)

        rate = rospy.get_param('~rate', 1.)
        rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def publish(self, event):
        histogram_msg = ColorHistogram()
        histogram_msg.header.stamp = rospy.Time.now()
        histogram_msg.header.frame_id = self.frame_id
        histogram_msg.histogram = self.histogram
        self.pub.publish(histogram_msg)

if __name__=='__main__':
    rospy.init_node("color_histogram_publisher")
    color_histogram_publisher = ColorHistogramPublisher()
    rospy.spin()
