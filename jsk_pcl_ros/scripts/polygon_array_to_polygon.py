#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray

def polygon_array_cb(plolygon_array):
    if (len(plolygon_array.polygons)==0):
        rospy.loginfo("polygon size 0")
        return
    PPub.publish(plolygon_array.polygons[0])


if __name__ == "__main__":
    rospy.init_node('polygon_array_to_polygon', anonymous=True)
    PPub = rospy.Publisher('polygon', PolygonStamped)
    rospy.Subscriber("polygon_array", PolygonArray, polygon_array_cb)
    rospy.spin()
