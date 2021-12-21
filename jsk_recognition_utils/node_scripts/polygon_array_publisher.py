#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Point32, PolygonStamped
from jsk_recognition_msgs.msg import ModelCoefficientsArray
from jsk_recognition_msgs.msg import PolygonArray
from pcl_msgs.msg import ModelCoefficients


def is_planar(points):
    points = np.array(points)
    # 3 points are always in a plane.
    if len(points) < 4:
        return True
    # 2d points are always in a plane.
    if len(points[0]) < 3:
        return True
    p0 = points[0]
    points = points[1:]
    dists = np.linalg.norm(points - p0, axis=1, ord=2)
    farthest_index = np.argmax(dists)
    p1 = points[farthest_index]
    v0 = p1 - p0
    p2 = points[np.argmin(np.dot(points - p0, v0))]
    v1 = p2 - p0
    normal = np.cross(v0, v1)
    norm = np.linalg.norm(normal, ord=2)
    normal = normal / norm
    dists = np.abs(np.dot(points - p0, normal))
    d = - np.dot(normal, p0)
    coeffs = np.array([normal[0], normal[1], normal[2], d], 'f')
    return np.allclose(dists, 0.0), coeffs


class PolygonArrayPublisher(object):
    def __init__(self):
        publish_rate = rospy.get_param("~publish_rate", 1.0)
        self.publish_coeffs = rospy.get_param("~publish_coeffs", False)

        frame_id = rospy.get_param("~frame_id")
        param = rospy.get_param("~polygons")
        self.are_polygons_in_planar = True
        self.msg, self.coeffs_array_msg = self.parse_params(param, frame_id)

        self.pub_polygons = rospy.Publisher("~output", PolygonArray, queue_size=1)
        if self.publish_coeffs:
            if self.are_polygons_in_planar is False:
                rospy.logwarn(
                    '~publish_coeffs is True,'
                    ' but ModelCoefficientsArray is not published'
                    ' because each polygon is not in planar.')
            else:
                self.pub_coeffs = rospy.Publisher(
                    '~output/coefficients',
                    ModelCoefficientsArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / publish_rate), self.publish)

    def parse_params(self, param, frame_id):
        # validate params
        assert isinstance(param, list), "polygons must be list"

        has_label = any("label" in p for p in param)
        has_likelihood = any("likelihood" in p for p in param)

        for polygon in param:
            assert "points" in polygon and isinstance(polygon["points"], list),\
                   "each polygon must have at least 1 'points'"
            for point in polygon["points"]:
                assert len(point) == 3,\
                       "element of 'points' must be list of 3 numbers"
            if has_label:
                assert "label" in polygon, "missing 'label'"
            if has_likelihood:
                assert "likelihood" in polygon, "missing 'likelihood'"

        # parse params
        msg = PolygonArray()
        msg.header.frame_id = frame_id
        coeffs_array_msg = ModelCoefficientsArray(header=msg.header)
        for polygon in param:
            ps = PolygonStamped()
            ps.header.frame_id = frame_id
            ps.polygon.points = [Point32(*v) for v in polygon["points"]]

            coeffs_msg = ModelCoefficients(header=ps.header)
            planar, coeffs = is_planar(polygon["points"])
            coeffs_msg.values = coeffs
            self.are_polygons_in_planar &= planar
            coeffs_array_msg.coefficients.append(coeffs_msg)
            msg.polygons.append(ps)
            if has_label:
                msg.labels.append(polygon["label"])
            if has_likelihood:
                msg.likelihood.append(polygon["likelihood"])
        return msg, coeffs_array_msg

    def publish(self, event):
        # update timestamp
        self.msg.header.stamp = event.current_real
        for p in self.msg.polygons:
            p.header.stamp = event.current_real

        self.pub_polygons.publish(self.msg)
        if self.publish_coeffs:
            self.coeffs_array_msg.header.stamp = event.current_real
            for p in self.coeffs_array_msg.coefficients:
                p.header.stamp = event.current_real
            self.pub_coeffs.publish(self.coeffs_array_msg)


if __name__ == '__main__':
    rospy.init_node("polygon_array_publisher")
    p = PolygonArrayPublisher()
    rospy.spin()
