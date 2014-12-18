#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import message_filters
import numpy as np
from tf.transformations import *

###########################################################
# utility function to colorize terminal output
def getcolor(colorname):
    colors = {
        'clear': '\033[0m',
        'black': '\033[30m',
        'red': '\033[31m',
        'green': '\033[32m',
        'yellow': '\033[33m',
        'blue': '\033[34m',
        'purple': '\033[35m',
        'cyan': '\033[36m',
        'white': '\033[37m'
        }
    def func2(c):
      return colors[colorname] + c + colors['clear']

    return func2

black  = getcolor('black')
red    = getcolor('red')
green  = getcolor('green')
yellow = getcolor('yellow')
blue   = getcolor('blue')
purple = getcolor('purple')
cyan   = getcolor('cyan')
white  = getcolor('white')
############################################################

# global variables
sub_checker_pose_a = None
sub_checker_pose_b = None
checker_pose_sync = None

def subscribeCheckerPoses(callback):
    global checker_pose_sync, sub_checker_pose_a, sub_checker_pose_b
    sub_checker_pose_a = message_filters.Subscriber('~checker_pose_a', PoseStamped)
    sub_checker_pose_b = message_filters.Subscriber('~checker_pose_b', PoseStamped)
    checker_pose_sync = message_filters.TimeSynchronizer([sub_checker_pose_a, sub_checker_pose_b], 10)
    checker_pose_sync.registerCallback(callback)

def subscribeCheckerPosesForEnvironment():
    subscribeCheckerPoses(checkerboardCallbackForEnvironment)

checker_board_poses_cache = []
def checkerboardCallbackForEnvironment(pose_a, pose_b):
    global checker_board_poses_cache
    # we need to compute 'static' transformation between pose_a and pose_b
    checker_board_poses_cache.append((pose_a, pose_b))

def unsubscribeCheckerPosesForEnvironment():
    global checker_pose_sync, sub_checker_pose_a, sub_checker_pose_b
    sub_checker_pose_a.sub.unregister()
    sub_checker_pose_b.sub.unregister()

def poseMsgFromPosRot(pos, rot):
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose
    
def posrotFromPoseMsg(pose_msg):
    return (np.array([pose_msg.position.x,
                      pose_msg.position.y,
                      pose_msg.position.z]),
            np.array([pose_msg.orientation.x,
                      pose_msg.orientation.y,
                      pose_msg.orientation.z,
                      pose_msg.orientation.w]))
    
def estimateCheckerPoseStaticalTransformation():
    global checker_board_poses_cache, checker_board_offset
    print "estimating from %d data" % (len(checker_board_poses_cache))
    if len(checker_board_poses_cache) == 0:
        raise Exception("There is no enough checkerboard data!")
    diff_positions = []
    diff_rotations = []
    for (pose_a, pose_b) in checker_board_poses_cache:
        (pose_a_pos, pose_a_rot) = posrotFromPoseMsg(pose_a.pose)
        (pose_b_pos, pose_b_rot) = posrotFromPoseMsg(pose_b.pose)
        # pose_b_pos + diff_pos = pose_a_pos
        diff_pos = np.dot(np.linalg.inv(quaternion_matrix(pose_b_rot)[:3, :3]),
                          pose_a_pos - pose_b_pos)
        # R_b * R = R_a
        # R = R_b^-1 * R_a
        diff_rot = quaternion_multiply(quaternion_inverse(pose_b_rot),
                                       pose_a_rot)
        diff_positions.append(diff_pos)
        diff_rotations.append(diff_rot)
    # compute mean
    mean_diff_pos = [0, 0, 0]
    mean_diff_rot = [0, 0, 0, 0]
    for pos in diff_positions:
        for i in range(3):
            mean_diff_pos[i] = mean_diff_pos[i] + pos[i]
    for rot in diff_rotations:
        for i in range(4):
            mean_diff_rot[i] = mean_diff_rot[i] + rot[i]
    mean_diff_pos = np.array(mean_diff_pos)
    mean_diff_rot = np.array(mean_diff_rot)
    mean_diff_pos = mean_diff_pos / len(diff_positions)
    mean_diff_rot = mean_diff_rot / len(diff_rotations)
    mean_diff_rot = mean_diff_rot / np.linalg.norm(mean_diff_rot)
    # stddev of pos
    std_x = np.std([diff[0] for diff in diff_positions])
    std_y = np.std([diff[1] for diff in diff_positions])
    std_z = np.std([diff[2] for diff in diff_positions])
    print "position stddev"
    print "x:: ", std_x
    print "y:: ", std_y
    print "z:: ", std_z
    checker_board_offset = (mean_diff_pos, mean_diff_rot)
    print checker_board_offset


latest_timestamp = None
def alreadyPoseARepublished(header):
    global latest_timestamp
    if not latest_timestamp or latest_timestamp < header.stamp:
        latest_timestamp = header.stamp
        return False
    else:
        return True
    
def poseARepublishCallback(msg):
    global pub_pose_a_republish
    if not alreadyPoseARepublished(msg.header):
        pub_pose_a_republish.publish(msg)

def matrixFromPosRot(pos, rot):
    # rot is quaternion
    mat = quaternion_matrix(rot)
    mat[0, 3] = pos[0]
    mat[1, 3] = pos[1]
    mat[2, 3] = pos[2]
    return mat
        
def estimatePoseARepublishCallback(msg):
    global pub_pose_a_republish, checker_board_offset
    # sleep a little bit to wait to use original pose a if possible
    #rospy.sleep(0.1)
    if not alreadyPoseARepublished(msg.header):
        (pos, rot) = posrotFromPoseMsg(msg.pose)
        (diff_pos, diff_rot) = checker_board_offset
        # diff_pos = np.array([0, -0.33, 0])
        # diff_rot = np.array([0, 0, 0, 1])
        a_rot = quaternion_multiply(rot, diff_rot)
        a_pos = pos + np.dot(quaternion_matrix(rot)[:3, :3], diff_pos)
        pub_msg = PoseStamped()
        pub_msg.header = msg.header
        pub_msg.pose = poseMsgFromPosRot(a_pos, a_rot)
        pub_pose_a_republish.publish(pub_msg)
        
def subscribeCheckerPosesForOcclusion():
    global sub_checker_pose_a, sub_checker_pose_b
    sub_checker_pose_a = rospy.Subscriber("~checker_pose_a", PoseStamped, 
                                          poseARepublishCallback)
    sub_checker_pose_b = rospy.Subscriber("~checker_pose_b", PoseStamped, 
                                          estimatePoseARepublishCallback)

def setupRepublisher():
    global pub_pose_a_republish
    pub_pose_a_republish = rospy.Publisher("~republished_pose_a", PoseStamped)
    
    
def captureEnvironment():
    print cyan("Please show environment and checker boards")
    print cyan("We will estimate checkerboard poses")
    subscribeCheckerPosesForEnvironment()
    raw_input(red("Please hit enter to stop detecting environment"))
    unsubscribeCheckerPosesForEnvironment()
    estimateCheckerPoseStaticalTransformation()
    setupRepublisher()
    subscribeCheckerPosesForOcclusion()
    raw_input(red("Please check the result of calibration"))
    

    
def captureObject():
    pass
    
def main():
    
    captureEnvironment()
    captureObject()

if __name__ == "__main__":
    rospy.init_node("capture_data_with_multi_checkerboard")
    main()
    
