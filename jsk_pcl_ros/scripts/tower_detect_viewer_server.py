#!/usr/bin/env python
# -*- coding: utf-8; -*-
"""
tower_detect_viewer_server.py

communicating with the browser and controlling the visualization

"""

import sys
import rospy

PKG='jsk_pcl_ros'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from image_view2.msg import ImageMarker2, PointArrayStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Header
from jsk_pcl_ros.msg import Int32Stamped
from jsk_recognition_msgs.srv import *
import jsk_recognition_msgs.srv
import tf
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
        self.publish()

        
class TowerDetectViewerServer:
    # name of tower
    TOWER_LOWEST = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.TOWER_LOWEST
    TOWER_MIDDLE = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.TOWER_MIDDLE
    TOWER_HIGHEST = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.TOWER_HIGHEST
    PLATE_SMALL = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.PLATE_SMALL
    PLATE_MIDDLE = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.PLATE_MIDDLE
    PLATE_LARGE = jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.PLATE_LARGE
    PLATE_HEIGHT_LOWEST = 0
    PLATE_HEIGHT_MIDDLE = 1
    PLATE_HEIGHT_HIGHEST = 2
    ROBOT0_BASE_FRAME_ID = "/R1/L0"
    ROBOT1_BASE_FRAME_ID = "/R2/L0"
    def __init__(self):
        # initialize the position of the tower
        self.tower_position = {
            self.TOWER_LOWEST: Point(),
            self.TOWER_MIDDLE: Point(),
            self.TOWER_HIGHEST: Point()
        }
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
        self.cluster_num = -1
        self.circle0.advertise()
        self.circle1.advertise()
        self.circle2.advertise()
        self.bridge = CvBridge()
        self.state = State("/browser/state")
        self.tf_listener = tf.TransformListener()
        self.browser_click_sub = rospy.Subscriber("/browser/click", 
                                                  Point, 
                                                  self.clickCB)
        self.browser_message_pub = rospy.Publisher("/browser/message",
                                                  String)
        self.image_sub = rospy.Subscriber("/image_marked",
                                          Image,
                                          self.imageCB)
        self.cluster_num_sub = rospy.Subscriber("/pcl_nodelet/clustering/cluster_num",
                                                Int32Stamped,
                                                self.clusterNumCB)
        self.check_circle_srv = rospy.Service("/browser/check_circle",
                                              CheckCircle,
                                              self.checkCircleCB)
        self.pickup_srv = rospy.Service("/browser/pickup",
                                        TowerPickUp,
                                        self.pickupCB)
        self.state.updateState(State.INITIAL)

        # waiting for ik server
        if rospy.get_param("~wait_robot_move_command", False):
            rospy.loginfo("waiting for robot server")
            rospy.wait_for_service("/browser/tower_robot_move_command")
        self.robot_command = rospy.ServiceProxy("/browser/tower_robot_move_command", TowerRobotMoveCommand)
        rospy.loginfo("connected to robot_move server")

        # initialize the position of the towers from TL
        self.updateTowerPosition(self.TOWER_LOWEST)
        self.updateTowerPosition(self.TOWER_MIDDLE)
        self.updateTowerPosition(self.TOWER_HIGHEST)
        self.S_TOWER = self.TOWER_MIDDLE
        self.G_TOWER = None
        self.I_TOWER = None
    def towerNameToFrameId(self, tower_name):
        if tower_name == self.TOWER_LOWEST:
            return "/cluster02"
        elif tower_name == self.TOWER_MIDDLE:
            return "/cluster01"
        elif tower_name == self.TOWER_HIGHEST:
            return "/cluster00"
        else:
            raise Exception("unknown tower: %d" % (tower_name))
    def resolveTowerName(self, tower_id):
        if tower_id == self.TOWER_LOWEST:
            return "TOWER_LOWEST"
        elif tower_id == self.TOWER_MIDDLE:
            return "TOWER_MIDDLE"
        elif tower_id == self.TOWER_HIGHEST:
            return "TOWER_HIGHEST"
        else:
            raise Exception("unknown tower: %d" % (tower_id))
    def resolvePlateName(self, plate_id):
        if plate_id == self.PLATE_SMALL:
            return "PLATE_SMALL"
        elif plate_id == self.PLATE_MIDDLE:
            return "PLATE_MIDDLE"
        elif plate_id == self.PLATE_LARGE:
            return "PLATE_LARGE"
        else:
            raise Exception("unknown plate id: %d" % (plate_id))
    def resolvePlateHeightOffset(self, height_id):
        """
        return the offset of z-axis of `height_id'
        """
        return 0.0
    def resolvePlateHeight(self, height_id):
        if height_id == self.PLATE_HEIGHT_LOWEST:
            return "lowest"
        elif height_id == self.PLATE_HEIGHT_MIDDLE:
            return "middle"
        elif height_id == self.PLATE_HEIGHT_HIGHEST:
            return "highest"
        else:
            raise Exception("unknown plate height: %d" % (height_id))
    def robotBaseFrameId(self, index):    #index is 0 or 1
        if index == 0:
            return self.ROBOT0_BASE_FRAME_ID
        elif index == 1:
            return self.ROBOT1_BASE_FRAME_ID
        else:
            raise Exception("unknown index: %d" % (index))
    def updateTowerPosition(self, tower_name):
        frame_id = self.towerNameToFrameId(tower_name)
        rospy.loginfo("resolving %s" % (frame_id))
        try:
            (trans, rot) = self.tf_listener.lookupTransform("/origin", frame_id, rospy.Time(0))
            rospy.loginfo("%s => %s: (%f, %f, %f)" % ("/origin", frame_id, trans[0], trans[1], trans[2]))
            self.tower_position[tower_name].x = trans[0]
            self.tower_position[tower_name].y = trans[1]
            self.tower_position[tower_name].z = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("failed to lookup transform: %s => %s" % ("/origin", frame_id))
    def clusterNumCB(self, msg):
        self.cluster_num = msg.data
    def moveRobot(self, plate, from_tower, to_tower, from_height, to_height):
        robot_index = 0                   #use the 1st robot first
        robot_frame_id = self.robotBaseFrameId(robot_index)
        rospy.loginfo("moving: %s from %s(%s) to %s(%s)" % (self.resolvePlateName(plate), 
                                                            self.resolveTowerName(from_tower), self.resolvePlateHeight(from_height),
                                                            self.resolveTowerName(to_tower), self.resolvePlateHeight(to_height)))
        from_target_position = self.tower_position[from_tower]
        to_target_position = self.tower_position[to_tower]
        self.robot_command(jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.ROBOT1,
                           plate,
                           from_tower, to_tower,
                           jsk_recognition_msgs.srv.TowerRobotMoveCommandRequest.OPTION_NONE)
        # self.robot_server1(Header(), from_target_position, 0)
        # self.robot_server1(Header(), to_target_position, 1)
    def runMain(self):
        # first of all, resolve tf and store the position of the tower
        # but, we don't need to update `S' tower's position.
        # update the tf value
        self.updateTowerPosition(self.I_TOWER)
        self.updateTowerPosition(self.G_TOWER)
        self.state.updateState(State.MOVE_LARGE_S_G)
        self.moveRobot(self.PLATE_LARGE, 
                       self.S_TOWER, self.G_TOWER, 
                       self.PLATE_HEIGHT_HIGHEST, self.PLATE_HEIGHT_LOWEST)
        
        self.state.updateState(State.MOVE_MIDDLE_S_I)
        self.moveRobot(self.PLATE_MIDDLE,
                       self.S_TOWER, self.I_TOWER,
                       self.PLATE_HEIGHT_MIDDLE, self.PLATE_HEIGHT_LOWEST)
        
        self.state.updateState(State.MOVE_LARGE_G_I)
        self.moveRobot(self.PLATE_LARGE, 
                       self.G_TOWER, self.I_TOWER, 
                       self.PLATE_HEIGHT_LOWEST, self.PLATE_HEIGHT_MIDDLE)
        
        self.state.updateState(State.MOVE_SMALL_S_G)
        self.moveRobot(self.PLATE_SMALL, 
                       self.S_TOWER, self.G_TOWER, 
                       self.PLATE_HEIGHT_LOWEST, self.PLATE_HEIGHT_LOWEST)
        
        self.state.updateState(State.MOVE_LARGE_I_S)
        self.moveRobot(self.PLATE_LARGE, 
                       self.I_TOWER, self.S_TOWER, 
                       self.PLATE_HEIGHT_MIDDLE, self.PLATE_HEIGHT_LOWEST)
        
        self.state.updateState(State.MOVE_MIDDLE_I_G)
        self.moveRobot(self.PLATE_MIDDLE, 
                       self.I_TOWER, self.G_TOWER, 
                       self.PLATE_HEIGHT_LOWEST, self.PLATE_HEIGHT_MIDDLE)
        
        self.state.updateState(State.MOVE_LARGE_S_G)
        self.moveRobot(self.PLATE_LARGE, 
                       self.S_TOWER, self.G_TOWER, 
                       self.PLATE_HEIGHT_LOWEST, self.PLATE_HEIGHT_HIGHEST)
    def pickupCB(self, req):
        target_index = req.tower_index
        # first of all, resolveing S, I and G name binding
        # S is the START tower
        # I is the INTERMEDIATE tower
        # G is the GOAL tower
        self.G_TOWER = req.tower_index
        # lookup I
        self.I_TOWER = (set([self.TOWER_LOWEST, self.TOWER_MIDDLE, self.TOWER_HIGHEST]) 
                        - set([self.G_TOWER, self.S_TOWER])).pop()

        # update the position of the tower
        self.state.updateState(State.MOVE_LARGE_S_G)
        self.state.publish()
        self.runMain()
        self.state.updateState(State.INITIAL)
        # update S
        self.S_TOWER = self.G_TOWER
        return TowerPickUpResponse()
    def checkCircleCB(self, req):
        (width, height) = cv.GetSize(self.cv_image)
        x = int(width * req.point.x)
        y = int(height * req.point.y)
        click_index = -1
        if self.checkColor(self.cv_image[y, x], self.color_indices[0]):
            click_index = self.TOWER_HIGHEST
        elif self.checkColor(self.cv_image[y, x], self.color_indices[1]):
            click_index = self.TOWER_MIDDLE
        elif self.checkColor(self.cv_image[y, x], self.color_indices[2]):
            click_index = self.TOWER_LOWEST
        if click_index == self.S_TOWER:
            msg = "the tower the user clicked equals to the start tower"
            rospy.logerr(msg)
            return CheckCircleResponse(False, click_index, msg)
        else:
            return CheckCircleResponse(click_index != -1, click_index, "")
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
            click_index = self.TOWER_HIGHEST
        elif self.checkColor(self.cv_image[y, x], self.color_indices[1]):
            output_str = output_str + " cluster01 clicked"
            click_index = self.TOWER_MIDDLE
        elif self.checkColor(self.cv_image[y, x], self.color_indices[2]):
            output_str = output_str + " cluster02 clicked"
            click_index = self.TOWER_LOWEST
        self.browser_message_pub.publish(String(output_str))
    def imageCB(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError as e:
            print(e)
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
