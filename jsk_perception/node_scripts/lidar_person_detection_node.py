#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker
from dynamic_reconfigure.server import Server as ReconfigureServer
from jsk_perception.cfg import LidarPersonDetectionConfig as Config
from jsk_topic_tools import ConnectionBasedTransport

from dr_spaam_libs.detector import DRSpaamDetector


class LidarPersonDetectionNode(ConnectionBasedTransport):

    def __init__(self):
        super(LidarPersonDetectionNode, self).__init__()
        self.weight_file = rospy.get_param("~weight_file")
        self.stride = rospy.get_param("~stride", 1)
        self.detector_model = rospy.get_param("~detector_model", 'DR-SPAAM')
        self.panoramic_scan = rospy.get_param("~panoramic_scan", False)

        self._srv = ReconfigureServer(Config, self.config_callback)

        self._detector = DRSpaamDetector(
            self.weight_file,
            model=self.detector_model,
            gpu=False,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )

        # Publisher
        self._dets_pub = self.advertise(
            '~output', PoseArray, queue_size=1)
        self._rviz_pub = self.advertise(
            '~output/marker', Marker, queue_size=1)

    def config_callback(self, config, level):
        self.conf_thresh = config.conf_thresh
        return config

    def subscribe(self):
        self._scan_sub = rospy.Subscriber(
            '~input', LaserScan, self._scan_callback,
            queue_size=rospy.get_param('~queue_size', 1))

    def unsubscribe(self):
        self._scan_sub.unregister()

    def _scan_callback(self, msg):
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges)))

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        dets_xy, dets_cls, instance_mask, sim_matrix = self._detector(scan)

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        if self._dets_pub.get_num_connections() > 0:
            dets_msg = detections_to_pose_array(dets_xy, dets_cls)
            dets_msg.header = msg.header
            self._dets_pub.publish(dets_msg)

        if self._rviz_pub.get_num_connections() > 0:
            rviz_msg = detections_to_rviz_marker(dets_xy, dets_cls)
            rviz_msg.header = msg.header
            self._rviz_pub.publish(rviz_msg)


def detections_to_rviz_marker(dets_xy, dets_cls):
    """
    @brief     Convert detection to RViz marker msg. Each detection is marked as
               a circle approximated by line segments.
    """
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "dr_spaam_ros"
    msg.id = 0
    msg.type = Marker.LINE_LIST

    # set quaternion so that RViz does not give warning
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    msg.scale.x = 0.03  # line width
    # red color
    msg.color.r = 1.0
    msg.color.a = 1.0

    # circle
    r = 0.4
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    # to msg
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        for i in range(len(xy_offsets) - 1):
            # start point of a segment
            p0 = Point()
            p0.x = d_xy[0] + xy_offsets[i, 0]
            p0.y = d_xy[1] + xy_offsets[i, 1]
            p0.z = 0.0
            msg.points.append(p0)

            # end point
            p1 = Point()
            p1.x = d_xy[0] + xy_offsets[i + 1, 0]
            p1.y = d_xy[1] + xy_offsets[i + 1, 1]
            p1.z = 0.0
            msg.points.append(p1)

    return msg


def detections_to_pose_array(dets_xy, dets_cls):
    pose_array = PoseArray()
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        p = Pose()
        p.position.x = d_xy[0]
        p.position.y = d_xy[1]
        p.position.z = 0.0
        p.orientation.w = 1.0
        pose_array.poses.append(p)

    return pose_array


if __name__ == '__main__':
    rospy.init_node('lidar_person_detection_node')
    node = LidarPersonDetectionNode()  # NOQA
    rospy.spin()
