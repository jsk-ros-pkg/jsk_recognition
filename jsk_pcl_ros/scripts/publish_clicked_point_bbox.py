#!/usr/bin/env python

import rospy
import tf

from jsk_recognition_msgs.msg import *
from geometry_msgs.msg import *

def clicked_point_cb(msg):
    listener.waitForTransform('odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
    p_conv = listener.transformPoint('odom', msg)
    bbox_array = BoundingBoxArray()
    bbox_array.header = p_conv.header
    bbox = BoundingBox()
    bbox.header = p_conv.header
    bbox.pose = Pose(position=p_conv.point)
    bbox.dimensions.x = 1
    bbox.dimensions.y = 1
    bbox.dimensions.z = 1
    bbox_array.boxes.append(bbox)
    bbox_pub.publish(bbox_array)

rospy.init_node('clicked_bbox')
listener = tf.TransformListener()
point_sub = rospy.Subscriber('clicked_point', PointStamped, clicked_point_cb)
bbox_pub = rospy.Publisher(
    'bbox_with_clicked_point', BoundingBoxArray, queue_size=1)
rospy.spin()
