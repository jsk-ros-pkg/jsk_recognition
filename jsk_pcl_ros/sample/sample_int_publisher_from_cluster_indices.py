#!/usr/bin/env python

import time

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import Int32Stamped
import rospy


def cb(msg):
    out_msg = Int32Stamped()
    out_msg.header = msg.header
    out_msg.data = int(time.time() % len(msg.cluster_indices))
    pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('sample_int_publisher_from_cluster_indices')
    pub = rospy.Publisher('~output', Int32Stamped, queue_size=1)
    sub = rospy.Subscriber('~input', ClusterPointIndices, cb, queue_size=1)
    rospy.spin()
