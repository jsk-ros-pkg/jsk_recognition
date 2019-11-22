#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Convert HistogramWithRangeArray to HistogramWithRange
"""

from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import HistogramWithRange, HistogramWithRangeArray

import rospy

class HistogramWithRangeArrayUnwrapper(ConnectionBasedTransport):
    def __init__(self):
        super(HistogramWithRangeArrayUnwrapper, self).__init__()
        self.index = rospy.get_param('~index', 0)
        self._pub = self.advertise("~output", HistogramWithRange, queue_size=1)
    def subscribe(self):
        self._sub = rospy.Subscriber("~input", HistogramWithRangeArray, self.callback)
    def unsubscribe(self):
        self._sub.unregister()
    def callback(self, msg):
        if len(msg.histograms) > self.index:
            self._pub.publish(msg.histograms[self.index])

if __name__ == "__main__":
    rospy.init_node("unwrap_histogram_with_range_array")
    unwrapper = HistogramWithRangeArrayUnwrapper()
    rospy.spin()
    
