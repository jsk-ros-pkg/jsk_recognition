#!/usr/bin/env python

from dynamic_reconfigure.server import Server as ReconfigureServer
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from jsk_recognition_utils.color import labelcolormap
from jsk_recognition_utils.visualization_marker import make_human_marker
from jsk_recognition_utils.geometry import rotation_matrix_from_axis
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import PyKDL
import rospy
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_matrix as matrix2quaternion
import tf2_geometry_msgs
import tf2_ros
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from jsk_perception.cfg import LidarPersonDetectionConfig as Config

from dr_spaam_libs.detector import DRSpaamDetector


N = 256
colors = labelcolormap(N=N)


class LidarPersonDetectionNode(ConnectionBasedTransport):

    def __init__(self):
        super(LidarPersonDetectionNode, self).__init__()
        self.weight_file = rospy.get_param("~weight_file")
        self.stride = rospy.get_param("~stride", 1)
        self.base_link = rospy.get_param("~base_link", None)
        self.detector_model = rospy.get_param("~detector_model", 'DR-SPAAM')
        self.panoramic_scan = rospy.get_param("~panoramic_scan", False)

        self._srv = ReconfigureServer(Config, self.config_callback)
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._last_time = rospy.Time.now()
        self._duration_timeout = rospy.get_param('~duration_timeout', 0.05)

        self._tracker = TrackingExtension()
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
            '~output/markers', MarkerArray, queue_size=1)

    def config_callback(self, config, level):
        self.conf_thresh = config.conf_thresh
        self.color_alpha = config.color_alpha
        self.people_head_radius = config.people_head_radius
        self.people_body_radius = config.people_body_radius
        self.people_height = config.people_height
        return config

    def subscribe(self):
        self._scan_sub = rospy.Subscriber(
            '~input', LaserScan, self._scan_callback,
            queue_size=rospy.get_param('~queue_size', 1))

    def unsubscribe(self):
        self._scan_sub.unregister()

    def _scan_callback(self, msg):
        now = rospy.Time.now()
        if now < self._last_time:
            rospy.logwarn(
                "Detected jump back in time of {}s. Clearing TF buffer."
                .format((self._last_time - now).to_sec()))
            self._tf_buffer.clear()
            if self._tracker:
                del self._tracker
                self._tracker = TrackingExtension()
        self._last_time = now

        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges)))

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        dets_xy, dets_cls, instance_mask, sim_matrix = self._detector(scan)
        if self._tracker:
            self._tracker(dets_xy, dets_cls, instance_mask, sim_matrix)

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        if self._dets_pub.get_num_connections() > 0:
            tracks, tracks_cls, track_ids = self._tracker.get_tracklets(
                only_latest_track=True)
            dets_msg = detections_to_pose_array(
                tracks, tracks_cls, track_ids,
                conf_thresh=self.conf_thresh)
            dets_msg.header = msg.header
            self._dets_pub.publish(dets_msg)

        if self._rviz_pub.get_num_connections() > 0:
            marker_array = MarkerArray()
            marker_frame_id = msg.header.frame_id
            base_to_laser = None

            if self.base_link is not None:
                try:
                    base_to_laser = tf2_geometry_msgs.transform_to_kdl(
                        self._tf_buffer.lookup_transform(
                            # remove '/' for lookupTransform
                            self.base_link.lstrip('/'),
                            msg.header.frame_id.lstrip('/'),
                            msg.header.stamp,
                            timeout=rospy.Duration(self._duration_timeout)))
                    marker_frame_id = self.base_link
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn('{}'.format(e))
                    return

            for d_xy in dets_xy:
                if base_to_laser is not None:
                    d_xy = base_to_laser * PyKDL.Vector(
                        d_xy[0], d_xy[1], 0.0)
                color = colors[(len(marker_array.markers) // 2) % N]
                color = list(color) + [self.color_alpha]
                markers = make_human_marker(
                    pos=(d_xy[0], d_xy[1], 0.0),
                    head_radius=self.people_head_radius,
                    body_radius=self.people_body_radius,
                    height=self.people_height,
                    frame_id=marker_frame_id.lstrip('/'),
                    stamp=msg.header.stamp,
                    id=len(marker_array.markers),
                    color=color)
                marker_array.markers.extend(markers)
            self._rviz_pub.publish(marker_array)


def detections_to_pose_array(tracks, tracks_cls, track_ids,
                             conf_thresh=0.8):
    pose_array = PoseArray()
    for track, tc, track_id in zip(tracks, tracks_cls, track_ids):
        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        if tc < conf_thresh:
            continue
        if len(track) > 1:
            p = Pose()
            p.position.x = track[-1][0]
            p.position.y = track[-1][1]
            p.position.z = 0.0

            x_axis = np.array([track[-1][0] - track[-2][0],
                               track[-1][1] - track[-2][1],
                               0.0], 'f')
            z_axis = (0, 0, 1)
            if np.linalg.norm(x_axis, ord=2) == 0:
                # invalid x_axis. x_axis shoule not be 0 vector.
                p.orientation.w = 1.0
            else:
                rotation = np.eye(4)
                rotation[:3, :3] = rotation_matrix_from_axis(
                    x_axis, z_axis, axes='xz')
                q_xyzw = matrix2quaternion(rotation)
                p.orientation.x = q_xyzw[0]
                p.orientation.y = q_xyzw[1]
                p.orientation.z = q_xyzw[2]
                p.orientation.w = q_xyzw[3]
        else:
            p = Pose()
            p.position.x = track[0][0]
            p.position.y = track[0][1]
            p.position.z = 0.0
            p.orientation.w = 1.0
        pose_array.poses.append(p)

    return pose_array


if __name__ == '__main__':
    rospy.init_node('lidar_person_detection_node')
    node = LidarPersonDetectionNode()  # NOQA
    rospy.spin()
