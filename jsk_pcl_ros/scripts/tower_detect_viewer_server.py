#!/usr/bin/env python
# -*- coding: utf-8; -*-
"""
tower_detect_viewer_server.py

communicating with the browser and controlling the visualization

"""

import sys

import rospy
import roslib
roslib.load_manifest("jsk_pcl_ros")

from image_view2.msg import ImageMarker2, PointArrayStamped
from geometry_msgs.msg import Point

from draw_3d_circle import Drawer3DCircle

class TowerDetectViewerServer:
    def __init__(self):
        self.radius = rospy.get_param("radius", 0.075)
        self.circle0 = Drawer3DCircle("/image_marker", 1, "/cluster00",
                                      self.radius, [1, 0, 0])
        self.circle1 = Drawer3DCircle("/image_marker", 2, "/cluster01",
                                      self.radius, [0, 1, 0])
        self.circle2 = Drawer3DCircle("/image_marker", 3, "/cluster02",
                                      self.radius, [0, 0, 1])
        self.circle0.advertise()
        self.circle1.advertise()
        self.circle2.advertise()
    def spin(self):
        while not rospy.is_shutdown():
            self.circle0.publish()
            self.circle1.publish()
            self.circle2.publish()
            rospy.sleep(1.0)


def main():
    rospy.init_node("tower_detect_viewer_server")
    server = TowerDetectViewerServer()
    server.spin()


if __name__ == "__main__":
    main()
