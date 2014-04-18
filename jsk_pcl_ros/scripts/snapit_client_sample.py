#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

roslib.load_manifest("interactive_markers")
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import *
from jsk_pcl_ros.srv import *
from jsk_pcl_ros.msg import *
from std_msgs.msg import *
import tf
import numpy

plane_center_pose = tf.transformations.identity_matrix()
FRAME_ID = "/camera_rgb_optical_frame"
tf_listener = None
snapit_result = None
server = None
SIZE = 0.1
HEIGHT = SIZE
MODEL = "PLANE"

def processFeedback(feedback):
  global plane_center_pose, MODEL

  if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    return
  elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
    if feedback.menu_entry_id == 1:
      # snapit!!
      trans = snapit_result.transformation
      trans_matrix = tf.transformations.quaternion_matrix(numpy.array((trans.orientation.x,
                                                                       trans.orientation.y,
                                                                       trans.orientation.z,
                                                                       trans.orientation.w)))
      pos = numpy.array((trans.position.x, trans.position.y, trans.position.z, 1.0))
      new_pos = pos
      trans_matrix[0, 3] = new_pos[0]
      trans_matrix[1, 3] = new_pos[1]
      trans_matrix[2, 3] = new_pos[2]
      new_trans = numpy.dot(trans_matrix, plane_center_pose)
      new_pose = Pose()
      new_pose.position.x = new_trans[0, 3]
      new_pose.position.y = new_trans[1, 3]
      new_pose.position.z = new_trans[2, 3]
      q = tf.transformations.quaternion_from_matrix(new_trans)
      new_pose.orientation.x = q[0]
      new_pose.orientation.y = q[1]
      new_pose.orientation.z = q[2]
      new_pose.orientation.w = q[3]
      server.setPose("snapit", new_pose, Header(frame_id = FRAME_ID,
                                                stamp = rospy.Time.now()))
      server.applyChanges()
      plane_center_pose = new_trans
      return
    elif feedback.menu_entry_id == 2:
      # changing the model
      if MODEL == "PLANE":
        MODEL = "CYLINDER"
      elif MODEL == "CYLINDER":
        MODEL = "PLANE"
      initializeMarker(server)
      menu_handler.apply( server, "snapit")
      server.applyChanges()
  elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
    pose = PoseStamped()
    pose.pose = feedback.pose
    pose.header = feedback.header
    transformed_pose = tf_listener.transformPose(FRAME_ID, pose)
    new_pose = tf.transformations.quaternion_matrix(numpy.array((
      transformed_pose.pose.orientation.x,
      transformed_pose.pose.orientation.y,
      transformed_pose.pose.orientation.z,
      transformed_pose.pose.orientation.w)))
    new_pose[0, 3] = transformed_pose.pose.position.x
    new_pose[1, 3] = transformed_pose.pose.position.y
    new_pose[2, 3] = transformed_pose.pose.position.z
    plane_center_pose = new_pose

def initializeMarker(server):
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = "/camera_rgb_optical_frame"
  int_marker.pose.position.y = 0
  int_marker.scale = 1

  int_marker.name = "snapit"
  
  marker = Marker()
  if MODEL == "PLANE":
    marker.type = Marker.CUBE
    marker.scale.x = SIZE * 2
    marker.scale.y = SIZE * 2
    marker.scale.z = 0.001
  elif MODEL == "CYLINDER":
    marker.type = Marker.CYLINDER
    marker.scale.x = SIZE
    marker.scale.y = SIZE
    marker.scale.z = HEIGHT
  marker.color.g = 1.0
  marker.color.a = 0.5
  control = InteractiveMarkerControl()
  control.always_visible = True
  control.markers.append( marker)
  int_marker.controls.append( control )
                  
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = "rotate_x"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = "move_x"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "rotate_z"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "move_z"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "rotate_y"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "move_y"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  server.insert(int_marker, processFeedback)

  
def main():
  global tf_listener, snapit_result, server, SIZE, menu_handler
  rospy.init_node("snapit_sample")
  rospy.wait_for_service("snapit")
  tf_listener = tf.TransformListener()
  s = rospy.ServiceProxy("snapit", CallSnapIt)
  plane_pub = rospy.Publisher("target_plane", PolygonStamped)
  server = InteractiveMarkerServer("snapit_plane")
  menu_handler = MenuHandler()
  menu_handler.insert("SnapIt", callback=processFeedback )
  menu_handler.insert("Change Model", callback=processFeedback )
  initializeMarker(server)
  menu_handler.apply( server, "snapit")
  server.applyChanges()
  
  # call snapit
  rospy.sleep(2)
  while not rospy.is_shutdown():
    req = SnapItRequest()
    req.header.stamp = rospy.Time.now()
    req.header.frame_id = "/camera_rgb_optical_frame"
    plane = PolygonStamped()
    plane.header.stamp = rospy.Time.now()
    plane.header.frame_id = "/camera_rgb_optical_frame"
    if MODEL == "PLANE":
      points = [numpy.array((SIZE, SIZE, 0.0, 1.0)),
                numpy.array((-SIZE, SIZE, 0.0, 1.0)),
                numpy.array((-SIZE, -SIZE, 0.0, 1.0)),
                numpy.array((SIZE, -SIZE, 0.0, 1.0))]
      for p in points:
        plane.polygon.points.append(Point32(*numpy.dot(plane_center_pose, p)[:3]))
      plane_pub.publish(plane)
      req.target_plane = plane
    elif MODEL == "CYLINDER":
      req.model_type = SnapItRequest.MODEL_CYLINDER
      req.center.header = req.header
      req.center.point.x = plane_center_pose[0, 3]
      req.center.point.y = plane_center_pose[1, 3]
      req.center.point.z = plane_center_pose[2, 3]
      z_axis = numpy.dot(plane_center_pose, numpy.array((0, 0, 1, 0)))
      req.direction.header = req.header
      req.direction.vector.x = z_axis[0]
      req.direction.vector.y = z_axis[1]
      req.direction.vector.z = z_axis[2]
      req.radius = SIZE
      req.height = HEIGHT
    try:
      snapit_result = s(req)
    except Exception, m:
      rospy.logerr("error in snapit")
      print m
    rospy.sleep(1)
  

if __name__ == "__main__":
  main()
