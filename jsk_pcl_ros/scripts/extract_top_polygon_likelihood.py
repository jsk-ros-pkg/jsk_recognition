#!/usr/bin/env python

import rospy 
from jsk_topic_tools import jsk_logdebug, jsk_loginfo, ConnectionBasedTransport
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray
from pcl_msgs.msg import ModelCoefficients
from geometry_msgs.msg import PolygonStamped
import message_filters

class ExtractTopPolygonLikelihood(ConnectionBasedTransport):
    def __init__(self):
        super(ExtractTopPolygonLikelihood, self).__init__()
        self._pub = self.advertise("~output", PolygonArray, queue_size=1)
        self._pub_coef = self.advertise("~output/coefficients", ModelCoefficientsArray, queue_size=1)
    def subscribe(self):
        self._sub = message_filters.Subscriber("~input", PolygonArray)
        self._sub_coef = message_filters.Subscriber("~input/coefficients", ModelCoefficientsArray)
        self._sync = message_filters.TimeSynchronizer([self._sub, self._sub_coef], 100)
        self._sync.registerCallback(self.callback)
    def unsubscribe(self):
        self._sub.sub.unregister()
        self._sub_coef.sub.unregister()
    def callback(self, msg, msg_coef):
        if len(msg.polygons) > 0:
            #self._pub.publish(msg.histograms[0])
            max_index = max(xrange(len(msg.polygons)), key=lambda i: msg.likelihood[i])
            res = PolygonArray()
            res.header = msg.header
            res.polygons = [msg.polygons[max_index]]
            res.likelihood = [msg.likelihood[max_index]]
            self._pub.publish(res)
            res_coef = ModelCoefficientsArray()
            res_coef.header = msg.header
            res_coef.coefficients = [msg_coef.coefficients[max_index]]
            self._pub_coef.publish(res_coef)


if __name__ == "__main__":
    rospy.init_node("extract_top_polygon_likelihood")
    ex = ExtractTopPolygonLikelihood()
    rospy.spin()
    

