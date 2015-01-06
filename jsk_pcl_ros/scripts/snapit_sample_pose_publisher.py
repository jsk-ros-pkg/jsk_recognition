#!/usr/bin/env python

import rospy
import numpy
from math import pi
from geometry_msgs.msg import PoseStamped, Pose
import tf
def main():
    convex_pose_pub = rospy.Publisher("/snapit/input/convex_align", PoseStamped)
    plane_pose_pub = rospy.Publisher("/snapit/input/plane_align", PoseStamped)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        x = (numpy.random.rand(1.0)[0] - 0.5) * 2.0
        y = (numpy.random.rand(1.0)[0] - 0.5) * 2.0
        z = (numpy.random.rand(1.0)[0] - 0.5) * 2.0
        # roll = (numpy.random.rand(1.0)[0] - 0.5) * pi
        # pitch = (numpy.random.rand(1.0)[0] - 0.5) * pi
        # yaw = (numpy.random.rand(1.0)[0] - 0.5) * pi
        roll = 0
        pitch = 0
        yaw = 0
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.header.stamp = rospy.Time.now()
        if numpy.random.rand(1.0) < 0.5:
            msg.header.frame_id = "odom"
        else:
            msg.header.frame_id = "odom"
        convex_pose_pub.publish(msg)
        plane_pose_pub.publish(msg)
        r.sleep()
        
if __name__ == "__main__":
    rospy.init_node("snapit_sample_pose_publisher")
    main()
