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
  print "Usage: ", sys.argv[0], "frame_id diameter [r, g, b]"

def main():
  if len(sys.argv) != 3 and len(sys.argv) != 6:
    usage()
    exit(1)
  frame_id = sys.argv[1]
  diameter = float(sys.argv[2])
  pub = rospy.Publisher("image_marker", ImageMarker2)
  rospy.init_node("draw_3d_circle")
  use_color = False
  color = ColorRGBA()
  if len(sys.argv) == 6:
    use_color = True
    color.r = float(sys.argv[3])
    color.g = float(sys.argv[4])
    color.b = float(sys.argv[5])
    color.a = 1.0
  while not rospy.is_shutdown():
    now = rospy.Time.now()
    marker = ImageMarker2()
    marker.header.frame_id = frame_id
    marker.header.stamp = now
    #marker.type = ImageMarker2.LINE_STRIP3D
    marker.type = ImageMarker2.POLYGON3D
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

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
