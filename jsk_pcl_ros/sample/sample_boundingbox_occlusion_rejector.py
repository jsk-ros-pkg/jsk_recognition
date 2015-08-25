#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped

candidate_pose = None

def candidatePoseCallback(msg):
    global candidate_pose
    candidate_pose = msg.pose
    
def callback(msg):
    if not candidate_pose:
        return
    target_array = BoundingBoxArray()
    target_array.header.stamp = msg.header.stamp
    target_array.header.frame_id = "world"
    target = BoundingBox()
    target.header.stamp = msg.header.stamp
    target.header.frame_id = "world"
    target.pose = candidate_pose
    target.dimensions.x = 0.2
    target.dimensions.y = 0.2
    target.dimensions.z = 0.2
    target_array.boxes = [target]
    pub_target.publish(target_array)
    candidate_array = BoundingBoxArray()
    candidate_array.header.stamp = msg.header.stamp
    candidate_array.header.frame_id = "world"
    for x in [-0.2, -0.1, 0.0, 0.1, 0.2]:
        for y in [-0.2, -0.1, 0.0, 0.1, 0.2]:
            for z in [-0.2, -0.1, 0.0, 0.1, 0.2]:
                candidate = BoundingBox()
                candidate.header.stamp = msg.header.stamp
                candidate.header.frame_id = "world"
                candidate.pose.position.z = 2 + z
                candidate.pose.position.x = x
                candidate.pose.position.y = y
                candidate.pose.orientation.w = 1.0
                candidate.dimensions.x = 0.1
                candidate.dimensions.y = 0.1
                candidate.dimensions.z = 0.1
                candidate_array.boxes.append(candidate)
    pub_candidate.publish(candidate_array)


if __name__ == "__main__":
    rospy.init_node("sample_boundingbox_occlusion_rejector")
    pub_target = rospy.Publisher("~output/target_boxes", BoundingBoxArray)
    pub_candidate = rospy.Publisher("~output/candidate_boxes", BoundingBoxArray)
    sub = rospy.Subscriber("~input", CameraInfo, callback)
    sub2 = rospy.Subscriber("~input/candidate_pose", PoseStamped, candidatePoseCallback)
    rospy.spin()
