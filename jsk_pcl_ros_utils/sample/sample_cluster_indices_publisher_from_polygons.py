#!/usr/bin/env python

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import PolygonArray
from pcl_msgs.msg import PointIndices
import rospy


def cb(msg):
    out_msg = ClusterPointIndices()
    out_msg.header = msg.header
    for _ in range(len(msg.polygons)):
        indices = PointIndices()
        out_msg.cluster_indices.append(indices)
    pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('sample_cluster_indices_publisher_from_polygons')
    pub = rospy.Publisher('~output', ClusterPointIndices, queue_size=1)
    sub = rospy.Subscriber('~input', PolygonArray, cb, queue_size=1)
    rospy.spin()
