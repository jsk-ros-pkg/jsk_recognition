#!/usr/bin/env python
import rospy
import numpy as np
PKG='jsk_pcl_ros'

import imp
## import message_filters
try:
    imp.find_module(PKG)
except:
    import roslib;roslib.load_manifest(PKG)

import math
from sensor_msgs.msg import Imu, CameraInfo, PointCloud2
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import ModelCoefficientsArray
from geometry_msgs.msg import *
from pcl_msgs.msg import ModelCoefficients
from std_msgs.msg import Header
header = Header()

def imu_cb(imu):
    ax, ay, az = imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z 
    # rospy.loginfo("%f %f %f" % (ax, ay, az))
    if az > 0.1:
        dx, dy = [-az, 0, ax], [0, -az, ay]
    elif ay > 0.1:
        dx, dy = [-ay, ax, 0], [0, az, -ay]
    else:
        dx, dy = [ay, -ax, 0], [az, 0, -az]
    PStamped = PolygonStamped()
    for x, y in [[-10, -10], [-10, 10], [10, 10], [10, -10]]:
        PStamped.polygon.points.append(Point32(x*dx[0]+y*dy[0],x*dx[1]+y*dy[1],x*dx[2]+y*dy[2]))
    temp_header = Header()
    temp_header.frame_id = imu.header.frame_id
    temp_header.stamp = header.stamp
    temp_header.seq = header.seq
    PStamped.header =temp_header
    PArrayPub.publish(PolygonArray(temp_header, [PStamped]))
    normal_array = np.array([ax, ay, az, 0])
    normal_array = normal_array / np.linalg.norm(x)
    MStamped = ModelCoefficients(temp_header, normal_array)
    MArrayPub.publish(ModelCoefficientsArray(temp_header, [MStamped]))
def camerainfo_cb(info):
    global header
    header = info.header
def point_cb(point):
    global header
    header = point.header
    
if __name__ == "__main__":
    rospy.init_node('calc_imu_plane', anonymous=True)
    PArrayPub = rospy.Publisher('polygon_array', PolygonArray)
    MArrayPub = rospy.Publisher('model_coefficients_array', ModelCoefficientsArray)
    rospy.Subscriber("imu_data", Imu, imu_cb)
    rospy.Subscriber("camera_info", CameraInfo, camerainfo_cb)
    rospy.Subscriber("points", PointCloud2, point_cb)
    rospy.spin()
