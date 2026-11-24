#!/usr/bin/env python
import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
import math
import tf
import numpy as np


def camera_info_cb(msg):
    global latest_camera_info
    latest_camera_info = msg;


def image_cb(msg):
    global latest_image
    latest_image = msg;


def cloud_cb(msg):
    global latest_camera_info, latest_image, frame_id
    # if latest_camera_info:
    if latest_image is not None:
    #     r = math.sqrt((msg.point.x - latest_camera_info.width/2)*(msg.point.x - latest_camera_info.width/2)+ (msg.point.y - latest_camera_info.height/2.0) * (msg.point.y - latest_camera_info.height/2.0))
        r = math.sqrt((msg.point.x - latest_image.width/2)*(msg.point.x - latest_image.width/2)+ (msg.point.y - latest_image.height/2.0) * (msg.point.y - latest_image.height/2.0))
        phi=r/341.0
        # x = -(msg.point.y - latest_camera_info.height/2.0) / r * math.sin(phi)
        # y = (msg.point.x - latest_camera_info.width/2.0) / r * math.sin(phi)
        x = -(msg.point.y - latest_image.height/2.0) / r * math.sin(phi)
        y = (msg.point.x - latest_image.width/2.0) / r * math.sin(phi)
        z = 1.0 * math.cos(phi)
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        first_vec = [x, -y,-z]
        second_vec = [1,0,0]
        dot = np.dot(first_vec, second_vec)/(tf.transformations.vector_norm(first_vec)*tf.transformations.vector_norm(second_vec))
        c = np.arccos(dot)

        M = tf.transformations.rotation_matrix(c,np.cross(first_vec, second_vec))
        quat = tf.transformations.quaternion_from_matrix(M)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        pose.header.frame_id=frame_id
        pub.publish(pose)

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id= frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = z
        pub_p.publish(point)

if __name__ == "__main__":
    rospy.init_node("click_to_pose")
    latest_image = None
    pub = rospy.Publisher("~output", PoseStamped, queue_size=1)
    pub_p = rospy.Publisher("~output_point", PointStamped, queue_size=1)
    frame_id = rospy.get_param("~frame_id", "fisheye")
    rospy.Subscriber("clicked_point", PointStamped, cloud_cb)
    rospy.Subscriber("camera_info", CameraInfo, camera_info_cb)
    rospy.Subscriber("image", Image, image_cb)
    rospy.spin()
