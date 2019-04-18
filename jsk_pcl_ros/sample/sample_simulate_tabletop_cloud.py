#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import (PolygonArray, ModelCoefficientsArray,
                                      BoundingBox, BoundingBoxArray)
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

def generatePointsDoor():
    plane_points = []
    object_points = []
    dx = 0.02
    for y in np.arange(-0.5, 0.5, dx):
        for z in np.arange(0, 2, dx):
            plane_points.append([0.0, y, z])
    for y in np.arange(0.2, 0.4, dx):
        for z in np.arange(0.8, 0.9, dx):
            object_points.append([-0.1, y, z])
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
    elif model_index == 4:
        return generatePointsDoor()

def generatePolygons(header, model_index):
    polygon = PolygonArray()
    polygon.header = header
    polygon.polygons = [PolygonStamped()]
    polygon.polygons[0].header = header;
    coef = ModelCoefficientsArray()
    coef.coefficients = [ModelCoefficients()]
    coef.header = header
    if model_index in [0, 1, 2, 3]:
        # Rectangle
        polygon.polygons[0].polygon.points = [Point32(x=1.0, y=1.0),
                                              Point32(x=-1.0, y=1.0),
                                              Point32(x=-1.0, y=-1.0),
                                              Point32(x=1.0, y=-1.0)]
        coef.coefficients[0].header = header
        coef.coefficients[0].values = [0, 0, 1, 0]
    elif model_index == 4:
        polygon.polygons[0].polygon.points = [Point32(x=0.0, y=-0.5, z=0.0),
                                              Point32(x=0.0, y=-0.5, z=2.0),
                                              Point32(x=0.0, y=0.5, z=2.0),
                                              Point32(x=0.0, y=0.5, z=0.0)]
        # polygon.polygons[0].polygon.points.reverse()
        coef.coefficients[0].header = header
        coef.coefficients[0].values = [-1, 0, 0, 0]
    return (polygon, coef)
    

def candidateBoxes(header, model_index):
    box_array = BoundingBoxArray()
    box_array.header.stamp = header.stamp
    box_array.header.frame_id = "odom"
    dx = 0.1
    for y in np.arange(0.0, 0.6, dx):
        for z in np.arange(0.7, 1.0, dx):
            for x in np.arange(0.0, -0.5, -dx):
                box = BoundingBox()
                box.header = box_array.header
                box.pose.orientation.w = 1.0
                box.pose.position.x = x
                box.pose.position.y = y
                box.pose.position.z = z
                box.dimensions.x = 0.1
                box.dimensions.y = 0.1
                box.dimensions.z = 0.1
                box_array.boxes.append(box)
    return box_array

if __name__ == "__main__":
    rospy.init_node("sample_simulate_tabletop_cloud")
    pub = rospy.Publisher("~output", PointCloud2, queue_size=1)
    pub_polygon = rospy.Publisher(
        "~output/polygon", PolygonArray, queue_size=1)
    pub_coef = rospy.Publisher(
        "~output/coef", ModelCoefficientsArray, queue_size=1)
    pub_boxes = rospy.Publisher(
        "~output/candidate_boxes", BoundingBoxArray, queue_size=1)
    r = rospy.Rate(10)
    counter = 0
    model_index = 4
    reset = False
    while not rospy.is_shutdown():
        points = generatePoints(model_index)
        header = Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()
        msg = create_cloud_xyz32(header, points)
        pub.publish(msg)
        (polygon, coef) = generatePolygons(header, model_index)
        pub_polygon.publish(polygon)
        pub_coef.publish(coef)
        counter = counter + 1
        pub_boxes.publish(candidateBoxes(header, model_index))
        if reset and counter > 1.0 / r.sleep_dur.to_sec() * 10:
            reset = rospy.ServiceProxy("/plane_supported_cuboid_estimator/reset", Empty)
            counter = 0
            model_index = model_index + 1
            if model_index >= 5:
                model_index = 0
            reset()
        r.sleep()
