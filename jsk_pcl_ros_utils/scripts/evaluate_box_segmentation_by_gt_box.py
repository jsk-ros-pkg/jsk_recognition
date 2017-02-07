#!/usr/bin/env python

from jsk_recognition_msgs.msg import Accuracy
from jsk_recognition_msgs.msg import BoundingBox
import jsk_recognition_utils
import rospy
from jsk_topic_tools import ConnectionBasedTransport


class EvaluateBoxSegmentationByGTBox(ConnectionBasedTransport):

    def __init__(self):
        super(EvaluateBoxSegmentationByGTBox, self).__init__()
        self.box_gt = None
        self.pub = self.advertise('~output', Accuracy, queue_size=1)
        self.sub_box_gt = rospy.Subscriber(
            '~input/box_gt', BoundingBox, self._cb_box_gt)

    def subscribe(self):
        self.sub_voxels = rospy.Subscriber(
            '~input/box', BoundingBox, self._cb_box)

    def unsubscribe(self):
        self.sub_voxels.unregister()

    def _cb_box_gt(self, box_msg):
        self.box_gt = jsk_recognition_utils.bounding_box_msg_to_aabb(box_msg)

    def _cb_box(self, box_msg):
        if self.box_gt is None:
            rospy.logwarn('GT. box is not set, skipping.')
            return
        box = jsk_recognition_utils.bounding_box_msg_to_aabb(box_msg)
        iu = jsk_recognition_utils.get_overlap_of_aabb(self.box_gt, box)

        out_msg = Accuracy(header=box_msg.header, accuracy=iu)
        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('evaluate_box_segmentation_by_gt_box')
    EvaluateBoxSegmentationByGTBox()
    rospy.spin()
