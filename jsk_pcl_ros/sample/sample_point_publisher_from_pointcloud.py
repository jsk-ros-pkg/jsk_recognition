#!/usr/bin/env python

from geometry_msgs.msg import PointStamped
import rospy
from sensor_msgs.msg import PointCloud2


def cb(msg):
    out_msg = PointStamped()
    out_msg.header = msg.header
    out_msg.point.x = pt_x
    out_msg.point.y = pt_y
    out_msg.point.z = pt_z
    pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('sample_point_publisher_from_pointcloud')
    pt_x = rospy.get_param('~x', 0.0)
    pt_y = rospy.get_param('~y', 0.0)
    pt_z = rospy.get_param('~z', 0.0)
    pub = rospy.Publisher('~output', PointStamped, queue_size=1)
    sub = rospy.Subscriber('~input', PointCloud2, cb, queue_size=1)
    rospy.spin()
