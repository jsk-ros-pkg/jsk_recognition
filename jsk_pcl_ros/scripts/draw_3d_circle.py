#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import sys
from image_view2.msg import ImageMarker2, PointArrayStamped
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

def usage():
  print("Usage: ", sys.argv[0], "id frame_id radius [r, g, b]")
  
class Drawer3DCircle:
  RESOLUTION = 20
  def __init__(self, topic, marker_id, frame_id, radius, color):
    self.topic = topic
    self.marker_id = marker_id
    self.frame_id = frame_id
    self.radius = radius
    self.fill = True
    if color:
      self.color = ColorRGBA()
      self.color.r = color[0]
      self.color.g = color[1]
      self.color.b = color[2]
  def advertise(self):
    self.publisher = rospy.Publisher(self.topic, ImageMarker2)
  def publish(self):
    marker = ImageMarker2()
    now = rospy.Time.now()
    marker.id = self.marker_id
    marker.header.stamp = now
    marker.header.frame_id = self.frame_id
    marker.type = ImageMarker2.POLYGON3D
    point_array = PointArrayStamped()
    point_array.header.frame_id = self.frame_id
    point_array.header.stamp = now
    for i in range(self.RESOLUTION + 1) + [0]:
      theta = 2 * math.pi / self.RESOLUTION * i
      x = self.radius * math.cos(theta)
      y = self.radius * math.sin(theta)
      point = Point()
      point.x = x
      point.y = y
      point.z = 0
      point_array.points.append(point)
    marker.points3D = point_array
    if self.color:
      marker.outline_colors = [self.color]
      if self.fill:
        marker.filled = 1
      else:
        marker.filled = 0
      marker.fill_color = self.color
    self.publisher.publish(marker)

def main():
  # global pub, marker_id, frame_id, radius, use_color, color
  rospy.init_node("draw_3d_circle")
  if len(rospy.myargv()) != 4 and len(rospy.myargv()) != 7:
    usage()
    exit(1)
  marker_id = int(rospy.myargv()[1])
  frame_id = rospy.myargv()[2]
  radius = float(rospy.myargv()[3])
  if len(rospy.myargv()) != 7:
    pub = Drawer3DCircle("image_marker", marker_id, frame_id, radius, None)
  else:
    pub = Drawer3DCircle("image_marker", marker_id, frame_id, radius, 
                         [float(rospy.myargv()[4]), float(rospy.myargv()[5]), float(rospy.myargv()[6])])
  pub.advertise()
  while not rospy.is_shutdown():
    pub.publish()
    rospy.sleep(1.0)
  
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
