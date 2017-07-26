#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from jsk_recognition_msgs.msg import PolygonArray
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header


def create_cloud_xyzrgb(header, xyzrgb):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgba', offset=16, datatype=PointField.UINT32, count=1),
    ]
    return create_cloud(header, fields, xyzrgb)


class TestPrimitiveClassifier(object):
    def __init__(self):
        self.pub_plane = rospy.Publisher("~output/plane", PolygonArray, queue_size=10)
        self.pub_cloud = rospy.Publisher("~output/cloud", PointCloud2, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def generate_plane(self, header,
                       xmin=-0.0, xmax=1.0,
                       ymin=-1.0, ymax=1.0):
        msg = PolygonArray()
        msg.header = header
        p = PolygonStamped()
        p.header = header
        p.polygon.points = [Point32(x=xmin, y=ymin),
                            Point32(x=xmax, y=ymin),
                            Point32(x=xmax, y=ymax),
                            Point32(x=xmin, y=ymax)]
        msg.polygons.append(p)
        return msg

    def generate_box_cloud(self, d=0.01, noise=0.0015,
                           xmin=0.2, xmax=0.4,
                           ymin=-0.05, ymax=0.05,
                           zmin=0.01, zmax=0.1):
        pts = []

        for x in np.arange(xmin, xmax, d):
            for y in np.arange(ymin, ymax, d):
                pts.append([x, y, zmax] + np.random.normal(0.0, noise, 3))

        for z in np.arange(zmin, zmax, d):
            for y in np.arange(ymin, ymax, d):
                pts.append([xmin, y, z] + np.random.normal(0.0, noise, 3))
        return pts

    def generate_circle_cloud(self, d=0.01, noise=0.0015,
                              radius=0.05,
                              xpos=0.2, ypos=0.2,
                              rmin=np.pi/2., rmax=np.pi*3./2,
                              zmin=0.01, zmax=0.15):
        pts = []

        for z in np.arange(zmin, zmax, d):
            for rad in np.arange(rmin, rmax, d*2.):
                pts.append([xpos + radius * np.cos(rad),
                            ypos + radius * np.sin(rad),
                            z] + np.random.normal(0.0, noise, 3))

        for r in np.arange(0, radius, d):
            for rad in np.arange(0, np.pi * 2., d*2.):
                pts.append([xpos + r * np.cos(rad),
                            ypos + r * np.sin(rad),
                            zmax] + np.random.normal(0.0, noise, 3))

        return pts

    def generate_plane_cloud(self, xmin, xmax, ymin, ymax):
        d = 0.02
        plane = []
        for x in np.arange(xmin, xmax, d):
            for y in np.arange(ymin, ymax, d):
                plane.append([x, y, 1.0])
        return plane

    def timer_cb(self, args=None):
        header = Header(frame_id="camera_rgb_optical_frame", stamp=rospy.Time.now())

        plane = self.generate_plane(header)
        self.pub_plane.publish(plane)

        pts = [pt.tolist() + [0xff0000ff] for pt in self.generate_box_cloud()]
        pts += [pt.tolist() + [0xffffffff] for pt in self.generate_circle_cloud()]
        cloud = create_cloud_xyzrgb(header, pts)
        self.pub_cloud.publish(cloud)


if __name__ == '__main__':
    rospy.init_node("test_primitive_classifier")
    c = TestPrimitiveClassifier()
    rospy.spin()
