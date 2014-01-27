#!/usr/bin/env python

import roslib
import rospy

roslib.load_manifest("jsk_pcl_ros")

from visualization_msgs.msg import Marker, MarkerArray

marker_array = None
def callback(msg):
  global marker_array, marker_array_pub
  if not marker_array:
    marker_array = MarkerArray()
  msg.id = len(marker_array.markers)
  marker_array.markers.append(msg)
  marker_array_pub.publish(marker_array)

def main():
  global marker_array_pub
  rospy.init_node("marker_appender")
  marker_array_pub = rospy.Publisher("marker_array", MarkerArray)
  s = rospy.Subscriber("marker", Marker, callback)
  rospy.spin()

if __name__ == "__main__":
  main()
