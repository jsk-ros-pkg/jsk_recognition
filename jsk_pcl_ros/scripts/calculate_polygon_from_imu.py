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

def imu_cb(imu):
    ax, ay, az = imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z 
    # rospy.loginfo("%f %f %f" % (ax, ay, az))
    if np.abs(az) > 0.1:
        dx, dy = [-az, 0, ax], [0, -az, ay]
    elif np.abs(ay) > 0.1:
        dx, dy = [-ay, ax, 0], [0, az, -ay]
    else:
        dx, dy = [ay, -ax, 0], [az, 0, -az]
    PStamped = PolygonStamped()
    for x, y in [[-10, -10], [-10, 10], [10, 10], [10, -10]]:
        PStamped.polygon.points.append(Point32(x*dx[0]+y*dy[0],x*dx[1]+y*dy[1],x*dx[2]+y*dy[2]))

    PStamped.header = imu.header
    PArrayPub.publish(PolygonArray(header=imu.header, polygons=[PStamped]))
    normal_array = np.array([ax, ay, az, 0])
    normal_array = normal_array / np.linalg.norm(x)
    MStamped = ModelCoefficients(imu.header, normal_array)
    MArrayPub.publish(ModelCoefficientsArray(imu.header, [MStamped]))
    
if __name__ == "__main__":
    rospy.init_node('calc_imu_plane', anonymous=True)
    PArrayPub = rospy.Publisher('polygon_array', PolygonArray, queue_size=1)
    MArrayPub = rospy.Publisher(
        'model_coefficients_array', ModelCoefficientsArray, queue_size=1)
    rospy.Subscriber("imu_data", Imu, imu_cb)
    rospy.spin()
