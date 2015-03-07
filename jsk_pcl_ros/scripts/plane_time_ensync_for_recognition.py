#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray


def input_polygons_cb(msg):
    global remain_polygon_msg
    remain_polygon_msg = msg
def input_coefficients_cb(msg):
    global remain_coefficients_msg
    remain_coefficients_msg = msg
def timer_cb(msg):
    global remain_polygon_msg, remain_coefficients_msg
    if remain_polygon_msg and remain_coefficients_msg:
        remain_polygon_msg.header.stamp = msg.header.stamp 
        remain_polygon_msg.header.seq = msg.header.seq
        remain_coefficients_msg.header.stamp = msg.header.stamp 
        remain_coefficients_msg.header.seq = msg.header.seq
        polygons_pub.publish(remain_polygon_msg)
        coefficients_pub.publish(remain_coefficients_msg)
        remain_polygon_msg = False
        remain_coefficients_msg = False

if __name__ == "__main__":
    rospy.init_node('time_ensync', anonymous=True)
    polygons_pub = rospy.Publisher('ensynced_planes', PolygonArray)
    coefficients_pub = rospy.Publisher('ensynced_planes_coefficients', ModelCoefficientsArray)
    rospy.Subscriber("timer", PointCloud2,  timer_cb)
    rospy.Subscriber("planes", PolygonArray,  input_polygons_cb)
    rospy.Subscriber("planes_coefficients", ModelCoefficientsArray,  input_coefficients_cb)
    remain_polygon_msg = False
    remain_coefficients_msg = False
    rospy.spin()
