#!/usr/bin/env python

from jsk_recognition_msgs.msg import BoundingBox
import rospy
from sensor_msgs.msg import PointCloud2


def cb(msg):
    out_msg = BoundingBox()
    out_msg.header.stamp = msg.header.stamp
    out_msg.header.frame_id = rospy.get_param('~frame_id', 'map')
    out_msg.pose.position.x = 0
    out_msg.pose.position.y = 0
    out_msg.pose.position.z = 0
    out_msg.pose.orientation.x = 0
    out_msg.pose.orientation.y = 0
    out_msg.pose.orientation.z = 0
    out_msg.pose.orientation.w = 1
    out_msg.dimensions.x = 0.3
    out_msg.dimensions.y = 0.3
    out_msg.dimensions.z = 0.3
    pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('sample_cluster_indices_publisher_from_polygons')
    pub = rospy.Publisher('~output', BoundingBox, queue_size=1)
    sub = rospy.Subscriber('~input', PointCloud2, cb, queue_size=1)
    rospy.spin()
