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
from std_msgs.msg import String
from draw_3d_circle import Drawer3DCircle

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv


class TowerDetectViewerServer:
    def __init__(self):
        self.radius = rospy.get_param("radius", 0.075)
        self.circle0 = Drawer3DCircle("/image_marker", 1, "/cluster00",
                                      self.radius, [1, 0, 0])
        self.circle1 = Drawer3DCircle("/image_marker", 2, "/cluster01",
                                      self.radius, [0, 1, 0])
        self.circle2 = Drawer3DCircle("/image_marker", 3, "/cluster02",
                                      self.radius, [0, 0, 1])
        # bgr
        self.color_indices = [[0, 0, 255], [0, 255, 0], [255, 0, 0]]
        self.circle0.advertise()
        self.circle1.advertise()
        self.circle2.advertise()
        self.bridge = CvBridge()
        self.browser_click_sub = rospy.Subscriber("/browser/click", 
                                                  Point, 
                                                  self.clickCB)
        self.browser_message_pub = rospy.Publisher("/browser/message",
                                                  String)
        self.image_sub = rospy.Subscriber("/image_marked",
                                          Image,
                                          self.imageCB)
    def checkColor(self, image_color, array_color):
        return (image_color[0] == array_color[0] and image_color[1] == array_color[1] and image_color[2] == array_color[2])
    def clickCB(self, msg):
        (width, height) = cv.GetSize(self.cv_image)
        # msg.x and msg.y is on a relative coordinate (u, v)
        x = int(width * msg.x)
        y = int(height * msg.y)
        output_str = str([x, y]) + " - " + str(self.cv_image[y, x])
        if self.checkColor(self.cv_image[y, x], self.color_indices[0]):
            output_str = output_str + " cluster00 clicked"
        elif self.checkColor(self.cv_image[y, x], self.color_indices[1]):
            output_str = output_str + " cluster01 clicked"
        elif self.checkColor(self.cv_image[y, x], self.color_indices[2]):
            output_str = output_str + " cluster02 clicked"
        self.browser_message_pub.publish(String(output_str))
    def imageCB(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
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
