#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel


class XYZToScreenPoint(object):
    def __init__(self):
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("~output", PointStamped, queue_size=1)

        rospy.Subscriber('~input/camera_info', CameraInfo, self.camera_info_cb)
        rospy.Subscriber('~input', PointStamped, self.point_stamped_cb)

    def camera_info_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.frame_id = msg.header.frame_id
        self.is_camera_arrived = True

    def point_stamped_cb(self, msg):
        if not self.is_camera_arrived:
            return
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = msg.point.x
        pose_stamped.pose.position.y = msg.point.y
        pose_stamped.pose.position.z = msg.point.z
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        position_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose.position
        pub_point = (position_transformed.x, position_transformed.y, position_transformed.z)
        u, v = self.cameramodels.project3dToPixel(pub_point)
        rospy.logdebug("u, v : {}, {}".format(u, v))
        pub_msg = PointStamped()
        pub_msg.header = msg.header
        pub_msg.header.frame_id = self.frame_id
        pub_msg.point.x = u
        pub_msg.point.y = v
        pub_msg.point.z = 0
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("xyz_to_screenpoint")
    xyz_to_screenpoint = XYZToScreenPoint()
    rospy.spin()
