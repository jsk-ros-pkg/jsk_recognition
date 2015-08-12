#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import PolygonArray, ModelCoefficientsArray
from pcl_msgs.msg import ModelCoefficients
from geometry_msgs.msg import PolygonStamped, Point32
from std_srvs.srv import Empty
import numpy as np
from math import pi, cos, sin

def generatePoints0():
    plane_points = []
    object_points = []
    dx = 0.02
    for x in np.arange(-1, 1, dx):
        for y in np.arange(-1, 1, dx):
            plane_points.append([x, y, 0.0])
    for x in np.arange(-0.1, 0.1, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([x, y, 0.2])
    return plane_points + object_points

def generatePoints1():
    plane_points = []
    object_points = []
    dx = 0.02
    for x in np.arange(-1, 1, dx):
        for y in np.arange(-1, 1, dx):
            plane_points.append([x, y, 0.0])
    for x in np.arange(-0.3, 0.2, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([x, y, 0.2])
    return plane_points + object_points

def generatePoints2():
    plane_points = []
    object_points = []
    dx = 0.02
    for x in np.arange(-1, 1, dx):
        for y in np.arange(-1, 1, dx):
            plane_points.append([x, y, 0.0])
    for x in np.arange(-0.1, 0.1, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([x, y, 0.2])
    for x in np.arange(-0.4, -0.2, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([x, y, 0.2])
    
    return plane_points + object_points

def generatePoints3():
    plane_points = []
    object_points = []
    dx = 0.02
    for x in np.arange(-1, 1, dx):
        for y in np.arange(-1, 1, dx):
            plane_points.append([x, y, 0.0])
    for x in np.arange(-0.1, 0.1, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([x, y, 0.2])
    for z in np.arange(0.0, 0.2, dx):
        for y in np.arange(-0.1, 0.1, dx):
            object_points.append([0.1, y, z])
            object_points.append([-0.1, y, z])
    for z in np.arange(0.0, 0.2, dx):
        for x in np.arange(-0.1, 0.1, dx):
            object_points.append([x, 0.1, z])
            object_points.append([x, -0.1, z])
    
    return plane_points + object_points

def generatePoints(model_index):
    if model_index == 0:
        return generatePoints0()
    elif model_index == 1:
        return generatePoints1()
    elif model_index == 2:
        return generatePoints2()
    elif model_index == 3:
        return generatePoints3()

def generatePolygons(header):
    polygon = PolygonArray()
    polygon.header = header
    polygon.polygons = [PolygonStamped()]
    polygon.polygons[0].header = header;
    # Rectangle
    polygon.polygons[0].polygon.points = [Point32(x=1.0, y=1.0), Point32(x=-1.0, y=1.0),
                                          Point32(x=-1.0, y=-1.0), Point32(x=1.0, y=-1.0)]
    # polygon.polygons[0].polygon.points = [Point32(x=2.0, y=1.0), Point32(x=-2.0, y=1.0),
    #                                       Point32(x=-2.0, y=-1.0), Point32(x=2.0, y=-1.0)]
    # circle
    # for i in range(100):
    #     theta = i / 100.0 * 2.0 * pi
    #     x = 1.0 * cos(theta)
    #     y = 1.0 * sin(theta)
    #     polygon.polygons[0].polygon.points.append(Point32(x=x, y=y))
    # star
    # polygon.polygons[0].polygon.points = [Point32(x= .0000, y= 1.0000),
    #                                       Point32(x= .2245, y= .3090),
    #                                       Point32(x= .9511, y= .3090),
    #                                       Point32(x= .3633, y= -.1180),
    #                                       Point32(x= .5878, y= -.8090),
    #                                       Point32(x= .0000, y= -.3820),
    #                                       Point32(x= -.5878, y= -.8090),
    #                                       Point32(x= -.3633, y= -.1180),
    #                                       Point32(x= -.9511, y= .3090),
    #                                       Point32(x= -.2245, y= .3090)]
    coef = ModelCoefficientsArray()
    coef.coefficients = [ModelCoefficients()]
    coef.header = header
    coef.coefficients[0].header = header
    coef.coefficients[0].values = [0, 0, 1, 0]
    return (polygon, coef)
    

if __name__ == "__main__":
    rospy.init_node("sample_simulate_tabletop_cloud")
    pub = rospy.Publisher("~output", PointCloud2)
    pub_polygon = rospy.Publisher("~output/polygon", PolygonArray)
    pub_coef = rospy.Publisher("~output/coef", ModelCoefficientsArray)
    r = rospy.Rate(10)
    counter = 0
    model_index = 2
    while not rospy.is_shutdown():
        points = generatePoints(model_index)
        header = Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()
        msg = create_cloud_xyz32(header, points)
        pub.publish(msg)
        (polygon, coef) = generatePolygons(header)
        pub_polygon.publish(polygon)
        pub_coef.publish(coef)
        counter = counter + 1
        if counter > 1.0 / r.sleep_dur.to_sec() * 10:
            reset = rospy.ServiceProxy("/plane_supported_cuboid_estimator/reset", Empty)
            counter = 0
            model_index = model_index + 1
            if model_index >= 4:
                model_index = 0
            reset()
        r.sleep()
