#!/usr/bin/env python

from jsk_topic_tools import ConnectionBasedTransport
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class MarkerToMarkerArray(ConnectionBasedTransport):

    def __init__(self):
        super(MarkerToMarkerArray, self).__init__()
        self.pub = self.advertise('~output', MarkerArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Marker, self.marker_cb)

    def unsubscribe(self):
        self.sub.unregister()

    def marker_cb(self, msg):
        array_msg = MarkerArray(markers=[msg])
        self.pub.publish(array_msg)


if __name__ == '__main__':
    rospy.init_node('marker_to_marker_array')
    MarkerToMarkerArray()
    rospy.spin()
