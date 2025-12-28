#!/usr/bin/env python
# -*- coding:utf-8 -*-

import dynamic_reconfigure.server
import geometry_msgs.msg
import jsk_recognition_msgs.msg
import message_filters
import numpy as np
import rospy
import shapely.geometry
from jsk_recognition_utils.cfg import PolygonArrayToBoxArrayConfig
from jsk_recognition_utils.geometry import rotation_matrix_from_axis
from jsk_topic_tools import ConnectionBasedTransport
from tf.transformations import quaternion_from_matrix as matrix2quaternion
from tf.transformations import unit_vector as normalize_vector


# minimum_rotated_rectangle is available since 1.6.0 (melodic)
# https://github.com/shapely/shapely/pull/361
from distutils.version import LooseVersion
if LooseVersion(shapely.__version__) < LooseVersion('1.6.0'):
    import math
    from itertools import islice
    from shapely.affinity import affine_transform
    @property
    def minimum_rotated_rectangle(polygon):
        """Returns the general minimum bounding rectangle of
        the geometry. Can possibly be rotated. If the convex hull
        of the object is a degenerate (line or point) this same degenerate
        is returned.
        """
        # first compute the convex hull
        hull = polygon.convex_hull
        try:
            coords = hull.exterior.coords
        except AttributeError: # may be a Point or a LineString
            return hull
        # generate the edge vectors between the convex hull's coords
        edges = ((pt2[0]-pt1[0], pt2[1]-pt1[1]) for pt1, pt2 in zip(coords, islice(coords, 1, None)))

        def _transformed_rects():
            for dx, dy in edges:
                # compute the normalized direction vector of the edge vector
                length = math.sqrt(dx**2 + dy**2)
                ux, uy = dx/length, dy/length
                # compute the normalized perpendicular vector
                vx, vy = -uy, ux
                # transform hull from the original coordinate system to the coordinate system
                # defined by the edge and compute the axes-parallel bounding rectangle
                transf_rect = affine_transform(hull, (ux,uy,vx,vy,0,0)).envelope
                # yield the transformed rectangle and a matrix to transform it back
                # to the original coordinate system
                yield (transf_rect, (ux,vx,uy,vy,0,0))

        # check for the minimum area rectangle and return it
        transf_rect, inv_matrix = min(_transformed_rects(), key=lambda r : r[0].area)
        return affine_transform(transf_rect, inv_matrix)
    shapely.geometry.Polygon.minimum_rotated_rectangle = minimum_rotated_rectangle


def angle_between_vectors(v1, v2, normalize=True,
                          directed=True):
    if normalize:
        v1 = normalize_vector(v1)
        v2 = normalize_vector(v2)
    dot = np.dot(v1, v2)
    return np.arccos(np.clip(dot if directed else np.fabs(dot), -1.0, 1.0))


def rotate_points(points, a, b):
    if points.ndim == 1:
        points = points[None, :]

    a = normalize_vector(a)
    b = normalize_vector(b)
    angle = angle_between_vectors(a, b, normalize=False, directed=False)
    if np.isclose(angle, 0.0):
        k = np.array([0, 0, 0], 'f')
    else:
        k = normalize_vector(np.cross(a, b))
    theta = angle_between_vectors(a, b, normalize=False)
    points_rot = points * np.cos(theta) \
        + np.cross(k, points) * np.sin(theta) \
        + k * np.dot(k, points.T).reshape(-1, 1) * (1 - np.cos(theta))
    return points_rot


class PolygonArrayToRectBoxArray(ConnectionBasedTransport):

    def __init__(self):
        super(PolygonArrayToRectBoxArray, self).__init__()
        dynamic_reconfigure.server.Server(
            PolygonArrayToBoxArrayConfig,
            self._config_callback)
        self.polygons_pub = self.advertise(
            '~output/polygons',
            jsk_recognition_msgs.msg.PolygonArray,
            queue_size=1)
        self.coeffs_pub = self.advertise(
            '~output/coefficients',
            jsk_recognition_msgs.msg.ModelCoefficientsArray,
            queue_size=1)
        self.boxes_pub = self.advertise(
            '~output/boxes',
            jsk_recognition_msgs.msg.BoundingBoxArray,
            queue_size=1)

    def _config_callback(self, config, level):
        self.thickness = config.thickness
        return config

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_polygon = message_filters.Subscriber(
            '~input/polygons',
            jsk_recognition_msgs.msg.PolygonArray,
            queue_size=1)
        sub_coefficients = message_filters.Subscriber(
            '~input/coefficients',
            jsk_recognition_msgs.msg.ModelCoefficientsArray,
            queue_size=1)
        self.subs = [sub_polygon, sub_coefficients]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb)

    def unsubscribe(self):
        for s in self.subs:
            s.unregister()

    def _cb(self, polygons_msg, coeffs_msg):
        new_polygons_msg = jsk_recognition_msgs.msg.PolygonArray()
        new_polygons_msg.header = polygons_msg.header
        boxes_msg = jsk_recognition_msgs.msg.BoundingBoxArray(
            header=polygons_msg.header)
        base_normal = [0, 0, 1]
        for coeff, polygon in zip(
                coeffs_msg.coefficients,
                polygons_msg.polygons):
            a, b, c, d = coeff.values
            points = np.array(
                [[point.x, point.y, point.z]
                 for point in polygon.polygon.points],
                dtype=np.float32)
            normal = [a, b, c]
            # Project 3d points onto a plane.
            projected_points = rotate_points(
                points,
                normal,
                base_normal)
            # Calculate the smallest rectangle on the plane.
            shapely_polygon = shapely.geometry.Polygon(
                projected_points)
            rect = shapely_polygon.minimum_rotated_rectangle
            x, y = rect.exterior.xy
            rect_polygon_2d = - np.ones((5, 3), 'f') * d
            rect_polygon_2d[:, 0] = x
            rect_polygon_2d[:, 1] = y
            # Return the rectangle to 3d space.
            rect_polygon = rotate_points(
                rect_polygon_2d,
                base_normal,
                normal)
            # Save as ros message.
            polygon = geometry_msgs.msg.PolygonStamped(
                header=polygon.header)
            for x, y, z in rect_polygon:
                polygon.polygon.points.append(
                    geometry_msgs.msg.Point32(x, y, z))
            new_polygons_msg.polygons.append(polygon)

            center_x, center_y, center_z = rect_polygon[:4].mean(axis=0)

            box_msg = jsk_recognition_msgs.msg.BoundingBox(
                header=polygon.header)
            base_matrix = np.eye(4)
            # The long side is the x-axis and
            # the normal direction is the z-axis.
            matrix = rotation_matrix_from_axis(
                rect_polygon[1] - rect_polygon[0], normal, 'xz')
            base_matrix[:3, :3] = matrix
            q_x, q_y, q_z, q_w = matrix2quaternion(base_matrix)
            dim_x = np.linalg.norm(rect_polygon[1] - rect_polygon[0])
            dim_y = np.linalg.norm(rect_polygon[2] - rect_polygon[1])
            box_msg.pose.position.x = center_x
            box_msg.pose.position.y = center_y
            box_msg.pose.position.z = center_z
            box_msg.pose.orientation.w = q_w
            box_msg.pose.orientation.x = q_x
            box_msg.pose.orientation.y = q_y
            box_msg.pose.orientation.z = q_z
            box_msg.dimensions.x = dim_x
            box_msg.dimensions.y = dim_y
            box_msg.dimensions.z = self.thickness
            boxes_msg.boxes.append(box_msg)
        self.polygons_pub.publish(new_polygons_msg)
        self.coeffs_pub.publish(coeffs_msg)
        self.boxes_pub.publish(boxes_msg)


if __name__ == '__main__':
    rospy.init_node('polygon_array_to_box_array')
    act = PolygonArrayToRectBoxArray()
    rospy.spin()
