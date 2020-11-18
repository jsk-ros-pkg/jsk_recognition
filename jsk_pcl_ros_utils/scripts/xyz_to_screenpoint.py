#!/usr/bin/env python

import rospy

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

from image_geometry import PinholeCameraModel

class XYZToScreenPoint:
    def __init__(self):
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False

    def fromCameraInfo_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.is_camera_arrived = True

    def point_stamped_cb(self, msg):
        if not self.is_camera_arrived:
            return
        point = (msg.point.x, msg.point.y, msg.point.z)
        u, v = self.cameramodels.project3dToPixel(point)
        rospy.logdebug("u, v : {}, {}".format(u, v))
        # publish info
        pub = rospy.Publisher("~output", PointStamped, queue_size=1)
        pub_msg = PointStamped()
        pub_msg.header = msg.header
        pub_msg.point.x = u
        pub_msg.point.y = v
        pub_msg.point.z = 0
        pub.publish(pub_msg)

    def subscribeCameraInfo(self):
        rospy.Subscriber('~input/camera_info', CameraInfo, self.fromCameraInfo_cb)
        rospy.Subscriber('~input', PointStamped, self.project3dToPixel_cb)


if __name__ == '__main__':
    rospy.init_node("xyz_to_screenpoint")
    xyz_to_screenpoint = XYZToScreenPoint()
    xyz_to_screenpoint.subscribeCameraInfo()
    rospy.spin()
