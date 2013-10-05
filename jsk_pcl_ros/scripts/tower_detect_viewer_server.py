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
from std_msgs.msg import Int16
from std_msgs.msg import String
from jsk_pcl_ros.srv import *

from draw_3d_circle import Drawer3DCircle

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv

class State:
    INITIAL = 1
    SELECT_TOWER = 2
    CONFIRM = 3
    START_TASK = 4
    INITIALIZE_PROBLEM = 5
    MOVE_LARGE_S_G = 6
    MOVE_MIDDLE_S_I = 7
    MOVE_LARGE_G_I = 8
    MOVE_SMALL_S_G = 9
    MOVE_LARGE_I_S = 10
    MOVE_MIDDLE_I_G = 11
    MOVE_LARGE_S_G = 12
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, Int16)
        self.state_val = -1
    def publish(self):
        self.pub.publish(Int16(self.state_val))
    def updateState(self, next_state):
        self.state_val = next_state

        
class TowerDetectViewerServer:
    def __init__(self):
        self.radius = rospy.get_param("radius", 0.075)
        self.circle0 = Drawer3DCircle("/image_marker", 1, "/cluster00",
                                      self.radius, [1, 0, 0])
        self.circle1 = Drawer3DCircle("/image_marker", 2, "/cluster01",
                                      self.radius, [0, 1, 0])
        self.circle2 = Drawer3DCircle("/image_marker", 3, "/cluster02",
                                      self.radius, [0, 0, 1])
        self.circles = [self.circle0, self.circle1, self.circle2]
        # bgr
        self.color_indices = [[0, 0, 255], [0, 255, 0], [255, 0, 0]]
        self.circle0.advertise()
        self.circle1.advertise()
        self.circle2.advertise()
        self.bridge = CvBridge()
        self.state = State("/browser/state")
        self.browser_click_sub = rospy.Subscriber("/browser/click", 
                                                  Point, 
                                                  self.clickCB)
        self.browser_message_pub = rospy.Publisher("/browser/message",
                                                  String)
        self.image_sub = rospy.Subscriber("/image_marked",
                                          Image,
                                          self.imageCB)
        self.check_circle_srv = rospy.Service("/browser/check_circle",
                                              CheckCircle,
                                              self.checkCircleCB)
        self.pickup_srv = rospy.Service("/browser/pickup",
                                        TowerPickUp,
                                        self.pickupCB)
        self.state.updateState(State.INITIAL)
    def pickupCB(self, req):
        self.state.updateState(State.MOVE_LARGE_S_G)
        self.state.publish()
        rospy.sleep(10)                   #適当
        self.state.updateState(State.INITIAL)
        self.state.publish()
        return TowerPickUpResponse()
    def checkCircleCB(self, req):
        (width, height) = cv.GetSize(self.cv_image)
        x = int(width * req.point.x)
        y = int(height * req.point.y)
        click_index = -1
        if self.checkColor(self.cv_image[y, x], self.color_indices[0]):
            click_index = 0
        elif self.checkColor(self.cv_image[y, x], self.color_indices[1]):
            click_index = 1
        elif self.checkColor(self.cv_image[y, x], self.color_indices[2]):
            click_index = 2
        return CheckCircleResponse(click_index != -1, click_index)
    def checkColor(self, image_color, array_color):
        return (image_color[0] == array_color[0] and 
                image_color[1] == array_color[1] and 
                image_color[2] == array_color[2])
    def clickCB(self, msg):
        (width, height) = cv.GetSize(self.cv_image)
        # msg.x and msg.y is on a relative coordinate (u, v)
        x = int(width * msg.x)
        y = int(height * msg.y)
        output_str = str([x, y]) + " - " + str(self.cv_image[y, x])
        click_index = -1
        if self.checkColor(self.cv_image[y, x], self.color_indices[0]):
            output_str = output_str + " cluster00 clicked"
            click_index = 0
        elif self.checkColor(self.cv_image[y, x], self.color_indices[1]):
            output_str = output_str + " cluster01 clicked"
            click_index = 1
        elif self.checkColor(self.cv_image[y, x], self.color_indices[2]):
            output_str = output_str + " cluster02 clicked"
            click_index = 2
        self.browser_message_pub.publish(String(output_str))
    def imageCB(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
    def publishState(self):
        self.state.publish()
    def spin(self):
        while not rospy.is_shutdown():
            for c in self.circles:
                c.publish()
            self.publishState()
            rospy.sleep(1.0)

def main():
    rospy.init_node("tower_detect_viewer_server")
    server = TowerDetectViewerServer()
    server.spin()


if __name__ == "__main__":
    main()
