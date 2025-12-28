#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import cv_bridge
import geometry_msgs.msg
from image_geometry.cameramodels import PinholeCameraModel
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_utils.geometry import rotation_matrix_from_axis
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg
import std_msgs.msg
from tf.transformations import quaternion_from_matrix


def area(poly):
    if len(poly) < 3:  # not a plane - no area
        return 0

    total = [0, 0, 0]
    for i in range(len(poly)):
        vi1 = poly[i]
        if i is len(poly) - 1:
            vi2 = poly[0]
        else:
            vi2 = poly[i + 1]
        prod = np.cross(vi1, vi2)
        total[0] += prod[0]
        total[1] += prod[1]
        total[2] += prod[2]
    result = np.dot(total, unit_normal(poly[0], poly[1], poly[2]))
    return abs(result / 2)


def unit_normal(a, b, c):
    x = np.linalg.det([[1, a[1], a[2]],
                       [1, b[1], b[2]],
                       [1, c[1], c[2]]])
    y = np.linalg.det([[a[0], 1, a[2]],
                       [b[0], 1, b[2]],
                       [c[0], 1, c[2]]])
    z = np.linalg.det([[a[0], a[1], 1],
                       [b[0], b[1], 1],
                       [c[0], c[1], 1]])
    magnitude = (x**2 + y**2 + z**2)**.5
    return (x / magnitude, y / magnitude, z / magnitude)


def angle(pt1, pt2, pt0):
    pt0 = np.array(pt0, dtype=np.float32)
    pt1 = np.array(pt1, dtype=np.float32)
    pt2 = np.array(pt2, dtype=np.float32)
    dx1 = pt1[0][0] - pt0[0][0]
    dy1 = pt1[0][1] - pt0[0][1]
    dx2 = pt2[0][0] - pt0[0][0]
    dy2 = pt2[0][1] - pt0[0][1]
    return (dx1 * dx2 + dy1 * dy2) / np.sqrt(
        (dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10)


class RectangleDetector(object):

    def __init__(self,
                 length_threshold=10,
                 distance_threshold=1.41421356,
                 canny_th1=50.0,
                 canny_th2=50.0,
                 canny_aperture_size=3,
                 do_merge=False):
        self.lsd = cv2.ximgproc.createFastLineDetector(
            length_threshold,
            distance_threshold,
            canny_th1,
            canny_th2,
            canny_aperture_size,
            do_merge)

    def find_squares(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        line_segments = self.lsd.detect(gray)

        if line_segments is None:
            line_segments = []
        edge_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        for line in line_segments:
            x1, y1, x2, y2 = map(int, line[0][:4])
            cv2.line(edge_img, (x1, y1), (x2, y2), (255, 255, 255), 10)

        _, contours, _ = cv2.findContours(
            edge_img,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE)

        squares = []
        count = 0
        for contour in contours:
            arclen = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour,
                                      arclen * 0.02,
                                      True)
            if len(approx) == 4:
                count += 1
            if (len(approx) == 4 and
                abs(cv2.contourArea(approx)) > 1000
                    and cv2.isContourConvex(approx)):
                maxCosine = 0
                for j in range(2, 5):
                    # find the maximum cosine of the angle between joint edges
                    cosine = abs(
                        angle(approx[j % 4], approx[j - 2], approx[j - 1]))
                    maxCosine = max(maxCosine, cosine)

                # if cosines of all angles are small
                # (all angles are ~90 degree) then write quandrange
                # vertices to resultant sequence
                if maxCosine < 0.3:
                    squares.append(approx)
                    approx = np.array(approx).reshape(-1, 2)
                    cv2.polylines(img,
                                  [approx],
                                  True,
                                  (0, 0, 255),
                                  thickness=3)
                    cv2.line(img, tuple(approx[0]), tuple(approx[2]),
                             (0, 0, 255), 1)
                    cv2.line(img, tuple(approx[1]), tuple(approx[3]),
                             (0, 0, 255), 1)
        return squares


def draw_squares(image, squares):
    for i in range(len(squares)):
        p = squares[i]
        p = np.array(p).reshape(-1, 2)
        image = cv2.polylines(image, [p], True, (0, 255, 0), thickness=3)


class PaperFinder(ConnectionBasedTransport):

    def __init__(self):
        super(PaperFinder, self).__init__()
        self.rectangle_detector = RectangleDetector()
        self.angle_tolerance = rospy.get_param(
            '~angle_tolerance', np.rad2deg(5.0))
        # 210mm * 297mm = 62370mm^2
        self.area_tolerance = rospy.get_param(
            '~area_tolerance', 0.1)
        self.rect_x = rospy.get_param(
            '~rect_x', 0.210)
        self.rect_y = rospy.get_param(
            '~rect_y', 0.297)
        self.area_size = self.rect_x * self.rect_y
        self.length_tolerance = rospy.get_param(
            '~length_tolerance', 0.04)
        self.image_pub = self.advertise('~output/viz',
                                        sensor_msgs.msg.Image,
                                        queue_size=1)
        self.pose_array_pub = self.advertise('~output/pose',
                                             geometry_msgs.msg.PoseArray,
                                             queue_size=1)
        self.bounding_box_array_pub = self.advertise(
            '~output/boxes', BoundingBoxArray, queue_size=1)
        self.length_array_pub = self.advertise(
            '~output/length', std_msgs.msg.Float32MultiArray, queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        self.camera_info_msg = None
        self.cameramodel = None

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        if rospy.get_param('~with_depth', True):
            sub_img = message_filters.Subscriber(
                '~input',
                sensor_msgs.msg.Image,
                queue_size=1,
                buff_size=2**24)
            sub_depth = message_filters.Subscriber(
                '~input/depth',
                sensor_msgs.msg.Image,
                queue_size=1,
                buff_size=2**24)
            self.subs = [sub_img, sub_depth]
            self.sub_info = rospy.Subscriber(
                '~input/camera_info',
                sensor_msgs.msg.CameraInfo, self._cb_cam_info)

            if rospy.get_param('~approximate_sync', True):
                slop = rospy.get_param('~slop', 0.1)
                sync = message_filters.ApproximateTimeSynchronizer(
                    fs=self.subs, queue_size=queue_size, slop=slop)
            else:
                sync = message_filters.TimeSynchronizer(
                    fs=self.subs, queue_size=queue_size)
            sync.registerCallback(self._cb_with_depth)
        else:
            sub = rospy.Subscriber(
                '~input',
                sensor_msgs.msg.Image,
                callback=self._cb,
                queue_size=queue_size)
            self.subs = [sub]

    def unsubscribe(self):
        for s in self.subs:
            s.unregister()

    def _cb_cam_info(self, msg):
        self.camera_info_msg = msg
        self.cameramodel = PinholeCameraModel()
        self.cameramodel.fromCameraInfo(msg)
        self.sub_info.unregister()
        self.sub_info = None
        rospy.loginfo("Received camera info")

    def _cb(self, msg):
        bridge = self.bridge
        try:
            cv_image = bridge.imgmsg_to_cv2(
                msg, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('{}'.format(e))
            return
        squares = self.rectangle_detector.find_squares(cv_image)

        if self.visualize:
            draw_squares(cv_image, squares)
            vis_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            vis_msg.header.stamp = msg.header.stamp
            self.image_pub.publish(vis_msg)

    def _cb_with_depth(self, img_msg, depth_msg):
        if self.camera_info_msg is None or self.cameramodel is None:
            rospy.loginfo("Waiting camera info ...")
            return
        bridge = self.bridge
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            depth_img = bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            if depth_msg.encoding == '16UC1':
                depth_img = np.asarray(depth_img, dtype=np.float32)
                depth_img /= 1000.0  # convert metric: mm -> m
            elif depth_msg.encoding != '32FC1':
                rospy.logerr('Unsupported depth encoding: %s' %
                             depth_msg.encoding)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('{}'.format(e))
            return
        squares = self.rectangle_detector.find_squares(cv_image)

        np_squares = np.array(squares, dtype=np.int32).reshape(-1, 2)

        height, width, _ = cv_image.shape
        # calculate xyz-position
        cameramodel = self.cameramodel
        x = (np_squares[:, 0] - cameramodel.cx()) / cameramodel.fx()
        y = (np_squares[:, 1] - cameramodel.cy()) / cameramodel.fy()
        z = depth_img.reshape(-1)[np_squares[:, 1] * width + np_squares[:, 0]]

        x *= z
        y *= z
        x = x.reshape(-1, 4, 1, 1)
        y = y.reshape(-1, 4, 1, 1)
        z = z.reshape(-1, 4, 1, 1)
        xyzs = np.concatenate([x, y, z], axis=3)
        new_squares = []
        valid_xyz_corners = []
        length_array_for_publish = std_msgs.msg.Float32MultiArray()
        for si, xyz in enumerate(xyzs):
            xyz_org = xyz.reshape(4, 3)
            xyz = np.concatenate([xyz, xyz], axis=0)
            valid = True
            for i in range(4):
                vec_a = xyz[i] - xyz[i + 1]
                vec_b = xyz[i + 2] - xyz[i + 1]
                zzz = np.inner(vec_a, vec_b)
                nnn = np.linalg.norm(vec_a) * np.linalg.norm(vec_b)
                if nnn == 0.0:
                    valid = False
                    break
                ccc = zzz / nnn
                calc_axis = np.arccos(np.clip(ccc, -1.0, 1.0))
                if abs(np.pi / 2.0 - calc_axis) > self.angle_tolerance:
                    valid = False
                    break
            if valid is False:
                break
            _a = (np.sqrt(np.sum((xyz_org[0] - xyz_org[1]) ** 2)))
            _b = (np.sqrt(np.sum((xyz_org[1] - xyz_org[2]) ** 2)))
            _c = (np.sqrt(np.sum((xyz_org[2] - xyz_org[3]) ** 2)))
            _d = (np.sqrt(np.sum((xyz_org[3] - xyz_org[0]) ** 2)))
            tmp = sorted([_a, _b, _c, _d])
            length_array = []
            length_array.append(_a)
            length_array.append(_b)
            length_array.append(_c)
            length_array.append(_d)
            length_array_for_publish = std_msgs.msg.Float32MultiArray(
                data=length_array)
            if abs(tmp[0] - self.rect_x) > self.length_tolerance or \
               abs(tmp[1] - self.rect_x) > self.length_tolerance or \
               abs(tmp[2] - self.rect_y) > self.length_tolerance or \
               abs(tmp[3] - self.rect_y) > self.length_tolerance:
                break
            area_value = area(xyz_org)
            if abs(self.area_size - area_value) < self.area_tolerance:
                new_squares.append(squares[si])
                valid_xyz_corners.append(xyz_org)
        self.length_array_pub.publish(length_array_for_publish)

        pose_array_msg = geometry_msgs.msg.PoseArray(header=img_msg.header)
        bounding_box_array_msg = BoundingBoxArray(header=img_msg.header)
        for xyz in valid_xyz_corners:
            center = np.mean(xyz, axis=0)

            vec_a = xyz[2] - xyz[1]
            vec_b = xyz[0] - xyz[1]
            if np.linalg.norm(vec_a) > np.linalg.norm(vec_b):
                vec_a, vec_b = vec_b, vec_a
            normal = np.cross(vec_a, vec_b)
            if normal[2] < 0:
                normal *= -1.0
            normal = normal / np.linalg.norm(normal)
            M = np.eye(4)
            M[:3, :3] = rotation_matrix_from_axis(
                normal, vec_a)
            q_xyzw = quaternion_from_matrix(M)
            _a = (np.sqrt(np.sum((xyz[0] - xyz[1]) ** 2)))
            _b = (np.sqrt(np.sum((xyz[1] - xyz[2]) ** 2)))
            _c = (np.sqrt(np.sum((xyz[2] - xyz[3]) ** 2)))
            _d = (np.sqrt(np.sum((xyz[3] - xyz[0]) ** 2)))
            lengths = np.array([_a, _b, _c, _d])
            indices = np.argsort(lengths)

            pose = geometry_msgs.msg.Pose()
            pose.position.x = center[0]
            pose.position.y = center[1]
            pose.position.z = center[2]
            pose.orientation.x = q_xyzw[0]
            pose.orientation.y = q_xyzw[1]
            pose.orientation.z = q_xyzw[2]
            pose.orientation.w = q_xyzw[3]
            pose_array_msg.poses.append(pose)
            bounding_box_array = BoundingBox(header=img_msg.header)
            bounding_box_array.pose.position.x = center[0]
            bounding_box_array.pose.position.y = center[1]
            bounding_box_array.pose.position.z = center[2]
            bounding_box_array.pose.orientation.x = q_xyzw[0]
            bounding_box_array.pose.orientation.y = q_xyzw[1]
            bounding_box_array.pose.orientation.z = q_xyzw[2]
            bounding_box_array.pose.orientation.w = q_xyzw[3]
            bounding_box_array.dimensions.x = 0.01
            bounding_box_array.dimensions.y = (
                lengths[indices[0]] + lengths[indices[1]]) / 2.0
            bounding_box_array.dimensions.z = (
                lengths[indices[2]] + lengths[indices[3]]) / 2.0
            bounding_box_array_msg.boxes.append(bounding_box_array)
        self.bounding_box_array_pub.publish(bounding_box_array_msg)
        self.pose_array_pub.publish(pose_array_msg)

        if self.visualize:
            draw_squares(cv_image, new_squares)
            vis_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            vis_msg.header.stamp = img_msg.header.stamp
            self.image_pub.publish(vis_msg)

    @property
    def visualize(self):
        return self.image_pub.get_num_connections() > 0


if __name__ == '__main__':
    rospy.init_node('paper_finder')
    act = PaperFinder()
    rospy.spin()
