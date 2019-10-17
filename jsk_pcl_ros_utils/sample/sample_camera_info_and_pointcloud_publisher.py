#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, PointCloud2

if __name__ == "__main__":
    rospy.init_node("sample_camera_info_and_pointcloud_publisher")
    pub_info = rospy.Publisher("~info", CameraInfo, queue_size=1)
    pub_cloud = rospy.Publisher("~cloud", PointCloud2, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        info = CameraInfo()
        info.header.stamp = rospy.Time.now()
        info.header.frame_id = "origin"
        info.height = 544
        info.width = 1024
        info.D = [-0.20831339061260223, 0.11341656744480133,
                  -0.00035378438769839704, -1.746419547998812e-05,
                  0.013720948249101639, 0.0, 0.0, 0.0]
        info.K = [598.6097412109375, 0.0, 515.5960693359375,
                  0.0, 600.0813598632812, 255.42999267578125,
                  0.0, 0.0, 1.0]
        info.R = [0.999993085861206, 0.0022128731943666935, -0.0029819998890161514,
                  -0.0022144035901874304, 0.9999974370002747, -0.0005100672133266926,
                  0.002980863442644477, 0.0005166670307517052, 0.9999954104423523]
        info.P = [575.3445434570312, 0.0, 519.5, 0.0,
                  0.0, 575.3445434570312, 259.5, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        pub_info.publish(info)
        cloud = PointCloud2()
        cloud.header.frame_id = "origin"
        pub_cloud.publish(cloud)
        rate.sleep()
