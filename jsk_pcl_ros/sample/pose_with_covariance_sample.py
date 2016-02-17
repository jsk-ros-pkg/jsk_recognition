#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, sin, pi

if __name__ == "__main__":
    rospy.init_node("pose_with_covariance_sample")
    pub = rospy.Publisher("~output", PoseWithCovarianceStamped)
    r = rospy.Rate(20)
    counter = 0
    rad = 2.0
    var = 0.3
    while not rospy.is_shutdown():
        counter = counter + 10
        theta = counter / 180.0 * pi
        if counter > 360:
            counter = 0
        var = abs(cos(theta / 2))
        x = rad * cos(theta)
        y = rad * sin(theta)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.covariance[0] = var
        msg.pose.covariance[7] = var
        msg.pose.covariance[14] = var
        msg.pose.covariance[21] = var
        msg.pose.covariance[28] = var
        msg.pose.covariance[35] = var
        pub.publish(msg)
        r.sleep()
