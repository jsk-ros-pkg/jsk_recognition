#!/usr/bin/env python3

try:
    from motpy import Detection, MultiObjectTracker
except ModuleNotFoundError:
    print('motpy is not found. Please install it.')
    import sys
    sys.exit(1)

import rospy
import message_filters
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import Object
from jsk_recognition_msgs.msg import ObjectArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from jsk_recognition_utils.panorama_utils import load_label_names

import numpy as np

import uuid


class BoundingBoxTracker(object):

    def __init__(self):

        super(BoundingBoxTracker, self).__init__()

        dt = rospy.get_param('~dt')
        min_iou = rospy.get_param('~min_iou', 0.1)
        multi_match_min_iou = rospy.get_param('~multi_match_min_iou', 1.000001)
        self.min_steps_alive = rospy.get_param('~min_steps_alive', 3)

        kwargs = {'min_iou': min_iou,
                  'multi_match_min_iou': multi_match_min_iou}


        model_preset = {'order_pos': 1,
                        'dim_pos': 3,
                        'order_size': 0,
                        'dim_size': 3}

        self.tracker = MultiObjectTracker(
                            dt=dt,
                            matching_fn_kwargs=kwargs,
                            model_spec=model_preset)

        self.target_labels = rospy.get_param('~target_labels', None)
        self.label_names = load_label_names()

        self.pub_bbox_array = rospy.Publisher('~output/bbox_array', BoundingBoxArray, queue_size=1)
        self.pub_obj_array = rospy.Publisher('~output/obj_array', ObjectArray, queue_size=1)

        self.subscribe()

    def subscribe(self):

        self.sub = rospy.Subscriber('~input', BoundingBoxArray, self.callback)

    def callback(self, bbox_array_msg):

        detections = []
        for bbox in bbox_array_msg.boxes:
            if self.target_labels is not None and \
                    self.label_names[bbox.label] not in self.target_labels:
                rospy.logwarn('{} is not in target_labels.'.format(self.label_names[bbox.label]))
                continue
            detections.append(
                Detection(box=np.array([
                                bbox.pose.position.x - bbox.dimensions.x / 2.0,
                                bbox.pose.position.y - bbox.dimensions.y / 2.0,
                                bbox.pose.position.z - bbox.dimensions.z / 2.0,
                                bbox.pose.position.x + bbox.dimensions.x / 2.0,
                                bbox.pose.position.y + bbox.dimensions.y / 2.0,
                                bbox.pose.position.z + bbox.dimensions.z / 2.0,
                                ]),
                          score=bbox.value,
                          class_id=bbox.label
                          )
            )

        self.tracker.step(detections=detections)
        active_tracks = self.tracker.active_tracks(min_steps_alive=self.min_steps_alive)

        output_bbox_array_msg = BoundingBoxArray()
        output_obj_array_msg = ObjectArray()

        output_bbox_array_msg.header = bbox_array_msg.header
        output_obj_array_msg.header = bbox_array_msg.header

        output_bbox_array_msg.boxes = [
                BoundingBox(
                    header=output_bbox_array_msg.header,
                    pose=Pose(
                        position=Point(
                            x=(active_track.box[0]+active_track.box[3])/2.0,
                            y=(active_track.box[1]+active_track.box[4])/2.0,
                            z=(active_track.box[2]+active_track.box[5])/2.0
                            ),
                        orientation=Quaternion(x=0,y=0,z=0,w=1.0)
                        ),
                    dimensions=Vector3(
                            x=(active_track.box[3]-active_track.box[0]),
                            y=(active_track.box[4]-active_track.box[1]),
                            z=(active_track.box[5]-active_track.box[2])
                            ),
                    label=(uuid.UUID(active_track.id).int % 2147483647),
                    )
                for active_track in active_tracks]
        output_obj_array_msg.objects = [
                Object(
                    id=(uuid.UUID(active_track.id).int % 2147483647),
                    class_id=active_track.class_id,
                    class_name=self.label_names[active_track.class_id],
                    )
                for active_track in active_tracks]

        if self.pub_bbox_array.get_num_connections() > 0 or self.pub_obj_array.get_num_connections() > 0:
            self.pub_bbox_array.publish(output_bbox_array_msg)
            self.pub_obj_array.publish(output_obj_array_msg)


if __name__ == '__main__':

    rospy.init_node('bounding_box_tracker_node')
    node = BoundingBoxTracker()
    rospy.spin()
