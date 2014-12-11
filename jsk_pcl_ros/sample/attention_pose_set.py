#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node("attention_pose_set")
pub = rospy.Publisher("/attention_clipper/input/pose", PoseStamped)
r = rospy.Rate(1)
while not rospy.is_shutdown():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/base_link"
    msg.pose.orientation.w = 1
    msg.pose.position.x = 0.8
    msg.pose.position.z = 1.5
    pub.publish(msg)
    r.sleep()
