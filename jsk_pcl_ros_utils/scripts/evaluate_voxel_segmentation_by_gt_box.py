#!/usr/bin/env python

from jsk_recognition_msgs.msg import Accuracy
from jsk_recognition_msgs.msg import BoundingBox
import jsk_recognition_utils
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from visualization_msgs.msg import MarkerArray


class EvaluateVoxelSegmentationByGTBox(ConnectionBasedTransport):

    def __init__(self):
        super(EvaluateVoxelSegmentationByGTBox, self).__init__()
        self.box_gt = None
        self.marker_ns = rospy.get_param('~marker_ns', None)
        if self.marker_ns is not None:
            self.marker_ns = str(self.marker_ns)
        self.pub = self.advertise('~output', Accuracy, queue_size=1)
        self.sub_box_gt = rospy.Subscriber(
            '~input/box_gt', BoundingBox, self._cb_box_gt)

    def subscribe(self):
        self.sub_voxels = rospy.Subscriber(
            '~input/markers', MarkerArray, self._cb_markers)

    def unsubscribe(self):
        self.sub_voxels.unregister()

    def _cb_box_gt(self, box_msg):
        self.box_gt = jsk_recognition_utils.bounding_box_msg_to_aabb(box_msg)

    def _cb_markers(self, markers_msg):
        if self.box_gt is None:
            rospy.logwarn('GT. box is not set, skipping.')
            return

        v_intersect = 0
        v_voxels = 0
        for marker in markers_msg.markers:
            if self.marker_ns and self.marker_ns != marker.ns:
                continue
            dim_x = marker.scale.x
            dim_y = marker.scale.y
            dim_z = marker.scale.z
            v_voxel = dim_x * dim_y * dim_z
            for point in marker.points:
                center_x = point.x
                center_y = point.y
                center_z = point.z
                x1 = center_x - dim_x / 2.
                x2 = center_x + dim_x / 2.
                y1 = center_y - dim_y / 2.
                y2 = center_y + dim_y / 2.
                z1 = center_z - dim_z / 2.
                z2 = center_z + dim_z / 2.
                voxel = (x1, y1, z1, x2, y2, z2)
                _, v_tp, _ = jsk_recognition_utils.get_overlap_of_aabb(
                    self.box_gt, voxel, return_volumes=True)
                v_intersect += v_tp
                v_voxels += v_voxel

        x1, y1, z1, x2, y2, z2 = self.box_gt
        v_box = (x2 - x1) * (y2 - y1) * (z2 - z1)

        v_union = v_voxels + v_box - v_intersect

        iu = 1. * v_intersect / v_union
        out_msg = Accuracy(header=markers_msg.markers[0].header, accuracy=iu)
        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('evaluate_voxel_segmentation_by_gt_box')
    EvaluateVoxelSegmentationByGTBox()
    rospy.spin()
