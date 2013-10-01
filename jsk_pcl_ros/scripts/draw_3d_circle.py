#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest("jsk_pcl_ros")
import sys
from image_view2.msg import ImageMarker2, PointArrayStamped
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

def usage():
  print "Usage: ", sys.argv[0], "id frame_id diameter timestamp_topic topic_type [r, g, b]"
  
pub = None
def callback(data):
  now = data.header.stamp
  marker = ImageMarker2()
  marker.id = marker_id
  marker.header.frame_id = frame_id
  marker.header.stamp = now
  marker.type = ImageMarker2.LINE_STRIP3D
  # marker.type = ImageMarker2.POLYGON3D
  RESOLUTION = 20
  point_array = PointArrayStamped()
  point_array.header.frame_id = frame_id
  point_array.header.stamp = now
  for i in range(RESOLUTION + 1) + [0]:
    # x = cos(theta), y = sin(theta)
    theta = 2 * math.pi / RESOLUTION * i
    x = diameter * math.cos(theta)
    y = diameter * math.sin(theta)
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    point_array.points.append(point)
  marker.points3D = point_array
  if use_color:
    marker.outline_colors = [color]
  pub.publish(marker)
  
def main():
  global pub, marker_id, frame_id, diameter, use_color, color
  rospy.init_node("draw_3d_circle")
  if len(rospy.myargv()) != 6 and len(rospy.myargv()) != 9:
    usage()
    exit(1)
  marker_id = int(rospy.myargv()[1])
  frame_id = rospy.myargv()[2]
  diameter = float(rospy.myargv()[3])
  pub = rospy.Publisher("image_marker", ImageMarker2)
  timestamp_topic = rospy.myargv()[4]
  topic_type = rospy.myargv()[5]          #like sensor_msgs/PointCloud2
  # loading module
  topic_module = topic_type.split("/")[0]
  topic_submodule = topic_type.split("/")[1]
  roslib.load_manifest(topic_module)
  #topic_type_object = getattr(__import__(topic_module + "." + "msg"), topic_submodule)
  topic_type_object = __import__(topic_module + "." + "msg")
  topic_type_class = getattr(topic_type_object.msg, topic_submodule)
  use_color = False
  color = ColorRGBA()
  if len(rospy.myargv()) == 9:
    use_color = True
    color.r = float(rospy.myargv()[6])
    color.g = float(rospy.myargv()[7])
    color.b = float(rospy.myargv()[8])
    color.a = 1.0
  rospy.Subscriber(timestamp_topic, topic_type_class, callback)
  rospy.spin()
  
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
