#!/usr/bin/env python

# program for recognition in hand
# depends on the node-structure of icp_registration
# suppose that robot does not move throughout recognizing

import rospy

PKG='jsk_pcl_ros'

import imp
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import PointsArray
from tf.transformations import *
import tf
from std_srvs import srv

teacher_pose_stamped = None
renew_flag = False
def pose_teacher_cb(pose_stamped):
    global teacher_pose_stamped
    teacher_pose_stamped = pose_stamped
    InputPosePub.publish(teacher_pose_stamped)
    # pub pose and save pose
def get_mat_from_pose(pose):
    return concatenate_matrices(
      translation_matrix([pose.position.x, pose.position.y, pose.position.z]),
      quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
      )
def get_pose_from_mat(mat):
    translation = translation_from_matrix(mat)
    rotation = quaternion_from_matrix(mat)
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = rotation[0]
    pose.orientation.y = rotation[1]
    pose.orientation.z = rotation[2]
    pose.orientation.w = rotation[3]
    return pose
    
def pose_diff_cb(pose_stamped):
    global teacher_pose_stamped, renew_flag
    # add diff and pub
    if (not teacher_pose_stamped):
        rospy.loginfo("teacher is empty")
        return
    try:
        teacher_pose_stamped.header.stamp = rospy.Time(0)
        teacher_pose_stamped_recog_frame = listener.transformPose(pose_stamped.header.frame_id, teacher_pose_stamped)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
        print("tf error: %s" % e)
        return
    teacher_pose_mat = (get_mat_from_pose(teacher_pose_stamped_recog_frame.pose))
    diff_pose_mat = (get_mat_from_pose(pose_stamped.pose))
    new_pose_mat = concatenate_matrices(diff_pose_mat, teacher_pose_mat)
    new_pose_stamped = PoseStamped()
    new_pose_stamped.pose = get_pose_from_mat(new_pose_mat)
    new_pose_stamped.header = pose_stamped.header
    OutputPosePub.publish(new_pose_stamped)
    # teachear_pose_stamped = None
    if renew_flag:
        try:
            new_pose_stamped.header.stamp = rospy.Time(0)
            new_pose_stamped_for_renew = listener.transformPose(teacher_pose_stamped.header.frame_id, new_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            print("tf error: %s" % e)
            return
        pose_teacher_cb(new_pose_stamped_for_renew)
        renew_flag = False
def renew_cb(req):
    global renew_flag
    renew_flag = True
    return srv.EmptyResponse()
if __name__ == "__main__":
    rospy.init_node("in_hand_recognition_manager")
    InputPosePub = rospy.Publisher(
        "~output/recognition", PoseStamped, queue_size=1)
    OutputPosePub = rospy.Publisher("~output", PoseStamped, queue_size=1)
    listener = tf.TransformListener()
    rospy.Subscriber("~input", PoseStamped, pose_teacher_cb)
    rospy.Subscriber("~input/result", PoseStamped, pose_diff_cb)
    rospy.Service("~renew", srv.Empty, renew_cb)
    rospy.spin()

