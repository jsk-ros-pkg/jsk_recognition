#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import PeoplePoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def find_pose(limb, msg):
    if isinstance(limb, list):
        for l in limb:
            p = find_pose(l, msg)
            if p is not None:
                return p
        return None
    try:
        idx = msg.limb_names.index(limb)
        return msg.poses[idx]
    except ValueError:
        return None


def remove_slash(frame_id):
    if frame_id.startswith("/"):
        return frame_id[1:]
    else:
        return frame_id


class PointIt(ConnectionBasedTransport):
    """
    PointIt class object
    """
    def __init__(self):
        super(PointIt, self).__init__()
        # variables
        self.subscribers = []
        self.objects = []

        # parameters
        if rospy.has_param("~dist_threshold"):
            rospy.logwarn("Parameter ~dist_threshold is deprecated. "
                          "Use ~max_dist_threshold instead.")
            self.max_dist_threshold = rospy.get_param("~dist_threshold")
        else:
            self.max_dist_threshold = rospy.get_param(
                "~max_dist_threshold", 0.1)

        self.min_dist_threshold = rospy.get_param("~min_dist_threshold", 0.0)
        self.min_norm_threshold = rospy.get_param("~min_norm_threshold", 0.2)
        self.use_arm = rospy.get_param("~use_arm", ["rarm", "larm"])

        if "rarm" not in self.use_arm and "larm" not in self.use_arm:
            rospy.logfatal("~use_arm must contain at least 'rarm' or 'larm'.")

        # tf listener
        use_tf2_buffer_client = rospy.get_param("~use_tf2_buffer_client", True)
        if use_tf2_buffer_client:
            self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
            if not self.tfl.wait_for_server(rospy.Duration(10)):
                rospy.logerr("Waiting /tf2_buffer_server timed out.")
                use_tf2_buffer_client = False
        if not use_tf2_buffer_client:
            self.tfl = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tfl)

        # advertise
        self.pub_bbox = self.advertise("~output", BoundingBoxArray, queue_size=1)
        self.pub_marker = self.advertise("~output/marker_array", MarkerArray, queue_size=1)

    def subscribe(self):
        use_cls = rospy.get_param("~use_classification_result", False)
        if use_cls:
            self.subscribers = [
                MF.Subscriber("~input/boxes", BoundingBoxArray, queue_size=1),
                MF.Subscriber("~input/class", ClassificationResult, queue_size=1),
            ]
            if rospy.get_param("~approximate_sync", True):
                sync = MF.ApproximateTimeSynchronizer(
                    self.subscribers,
                    queue_size=rospy.get_param("~queue_size", 10),
                    slop=rospy.get_param("~slop", 0.1),
                )
            else:
                sync = MF.TimeSynchronizer(
                    self.subscribers,
                    queue_size=rospy.get_param("~queue_size", 100),
                )
            sync.registerCallback(self.on_objects)
        else:
            self.subscribers = [
                rospy.Subscriber("~input/boxes", BoundingBoxArray,
                                 self.on_objects, queue_size=1),
            ]

        self.subscribers.append(
            rospy.Subscriber("~input", PeoplePoseArray, self.on_people, queue_size=1))

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def on_objects(self, bbox_msg, cls_msg=None):
        """
        bbox_msg (jsk_recognition_msgs/BoundingBoxArray)
        cls_msg (jsk_recognition_msgs/ClassificationResult)
        """
        if cls_msg is not None:
            if len(bbox_msg.boxes) != len(cls_msg.labels):
                rospy.logwarn("len(boxes) != len(labels)")
                return
            for bbox, label in zip(bbox_msg.boxes, cls_msg.labels):
                bbox.label = label
        self.objects = bbox_msg.boxes
        rospy.loginfo("on_objects")

    def on_people(self, ppl_msg):
        """
        ppl_msg (jsk_recognition_msgs/PeoplePoseArray)

        refer: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
        """
        frame_id = remove_slash(ppl_msg.header.frame_id)
        in_objects = self.objects
        objects = []
        for bbox in in_objects:
            if remove_slash(bbox.header.frame_id) != frame_id:
                ps = PoseStamped(header=bbox.header, pose=bbox.pose)
                try:
                    ps = self.tfl.transform(ps, frame_id)
                    bbox.header, bbox.pose = ps.header, ps.pose
                    objects.append(bbox)
                except tf2_ros.TransformException:
                    continue

        out_bboxes = []
        out_markers = []
        for i, pose in enumerate(ppl_msg.poses):
            # for each person
            if "rarm" in self.use_arm:
                rwrist = find_pose("RWrist", pose)
                rfinger = find_pose(["RHand5", "RHand6", "RHand7", "RHand8"], pose)
                if rwrist is not None and rfinger is not None:
                    rclosest, rdist = self.get_closest_bbox((rwrist, rfinger), objects)
                    if rdist is not None and\
                       self.min_dist_threshold <= rdist <= self.max_dist_threshold:
                        out_bboxes.append(rclosest)
                    rmarker = self.get_marker((rwrist, rfinger), frame_id)
                    rmarker.id = 2 * i
                    out_markers.append(rmarker)

            if "larm" in self.use_arm:
                lwrist = find_pose("LWrist", pose)
                lfinger = find_pose(["LHand5", "LHand6", "LHand7", "LHand8"], pose)
                if lwrist is not None and lfinger is not None:
                    lclosest, ldist = self.get_closest_bbox((lwrist, lfinger), objects)
                    if ldist is not None and\
                       self.min_dist_threshold <= ldist <= self.max_dist_threshold:
                        out_bboxes.append(lclosest)
                    lmarker = self.get_marker((lwrist, lfinger), frame_id)
                    lmarker.id = 2 * i + 1
                    out_markers.append(lmarker)

        # publish
        if out_bboxes:
            msg = BoundingBoxArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.boxes = out_bboxes
            self.pub_bbox.publish(msg)
        if out_markers:
            self.pub_marker.publish(MarkerArray(markers=out_markers))

    def get_marker(self, line_seg, frame_id):
        p0, p1 = line_seg
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = frame_id
        m.ns = "beam"
        m.type = Marker.ARROW
        m.action = Marker.MODIFY
        m.lifetime = rospy.Duration(1)
        m.points = [p0.position, p1.position]
        m.scale.x = 0.02
        m.scale.y = 0.04
        m.color.r = 1.0
        m.color.a = 1.0
        return m

    def get_closest_bbox(self, line_seg, bboxes):
        pose0, pose1 = line_seg
        p0 = np.asarray([pose0.position.x, pose0.position.y, pose0.position.z])
        p1 = np.asarray([pose1.position.x, pose1.position.y, pose1.position.z])
        min_d, closest = None, None

        n01 = np.linalg.norm(p1 - p0)
        if n01 == 0.0:
            return closest, min_d

        for bbox in bboxes:
            pos = bbox.pose.position
            p = np.asarray([pos.x, pos.y, pos.z])

            # check if the order is p0 -> p1 -> p
            n = np.linalg.norm(p - p0)
            ip = np.inner(p1 - p0, p - p0) / (n01 * n)
            rospy.logdebug("ip: %s, dn: %s" % (ip, n - n01))
            if ip < 0.0 or n - n01 < self.min_norm_threshold:
                continue

            d = np.linalg.norm(np.cross(p1 - p0, p0 - p)) / np.linalg.norm(p1 - p0)
            if min_d is None or min_d > d:
                min_d, closest = d, bbox

        return closest, min_d


if __name__ == '__main__':
    rospy.init_node("pointit")
    p = PointIt()
    rospy.spin()
