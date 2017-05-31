#!/usr/bin/env python

import rospy 
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray
from pcl_msgs.msg import ModelCoefficients
from geometry_msgs.msg import PolygonStamped
import message_filters
from dynamic_reconfigure.server import Server
from jsk_pcl_ros.cfg import ExtractTopPolygonLikelihoodConfig

class ExtractTopPolygonLikelihood(ConnectionBasedTransport):
    def __init__(self):
        super(ExtractTopPolygonLikelihood, self).__init__()
        self._srv = Server(ExtractTopPolygonLikelihoodConfig, self.config_callback)
        self._pub = self.advertise("~output", PolygonArray, queue_size=1)
        self._pub_coef = self.advertise("~output/coefficients", ModelCoefficientsArray, queue_size=1)
    def config_callback(self, config, level):
        self._min_likelihood = config.min_likelihood
        return config
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
            if msg.likelihood[max_index] < self._min_likelihood:
                rospy.loginfo("Ignore result because of too small likelihood: {0} < {1}".format(
                    msg.likelihood[max_index],
                    self._min_likelihood))
                return
            self._pub.publish(res)
            res_coef = ModelCoefficientsArray()
            res_coef.header = msg.header
            res_coef.coefficients = [msg_coef.coefficients[max_index]]
            self._pub_coef.publish(res_coef)


if __name__ == "__main__":
    rospy.init_node("extract_top_polygon_likelihood")
    ex = ExtractTopPolygonLikelihood()
    rospy.spin()
    

