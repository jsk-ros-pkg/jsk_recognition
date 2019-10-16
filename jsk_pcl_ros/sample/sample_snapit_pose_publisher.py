#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy


if __name__ == "__main__":
    rospy.init_node("sample_snapit_pose_publisher")
    frame_id = rospy.get_param('~frame_id', 'map')
    pub = rospy.Publisher("~output", PoseStamped, queue_size=1)
    r = rospy.Rate(rospy.get_param('~rate', 1.0))

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        theta = now % (2 * np.pi)

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.pose.position.x = 4 * np.cos(theta)
        msg.pose.position.y = 0
        msg.pose.position.z = 2 * np.sin(theta) + 1
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        pub.publish(msg)

        r.sleep()
