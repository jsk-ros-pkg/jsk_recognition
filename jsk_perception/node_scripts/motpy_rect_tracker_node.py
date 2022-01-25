#!/usr/bin/env python3

try:
    from motpy import Detection, MultiObjectTracker
except ModuleNotFoundError:
    print('motpy is not found. Please install it.')
    import sys
    sys.exit(1)

import rospy
import message_filters
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Track
from jsk_recognition_msgs.msg import TrackArray

from jsk_recognition_utils.panorama_utils import load_label_names

import numpy as np

import uuid


class MotpyRectTracker(object):

    def __init__(self):

        super(MotpyRectTracker, self).__init__()

        dt = rospy.get_param('~dt')
        min_iou = rospy.get_param('~min_iou', 0.1)
        multi_match_min_iou = rospy.get_param('~multi_match_min_iou', 1.000001)
        self.min_steps_alive = rospy.get_param('~min_steps_alive', 3)

        kwargs = {'min_iou': min_iou,
                  'multi_match_min_iou': multi_match_min_iou}

        self.tracker = MultiObjectTracker(
                            dt=dt,
                            matching_fn_kwargs=kwargs)

        self.target_labels = rospy.get_param('~target_labels', [])
        self.label_names = load_label_names()

        self.pub_tracks = rospy.Publisher(
            '~output/tracks', TrackArray, queue_size=1)

        self.subscribe()

    def subscribe(self):

        queue_size = rospy.get_param('~queue_size', 100)
        sub_rects = message_filters.Subscriber(
            '~input/rects', RectArray, queue_size=1)
        sub_class = message_filters.Subscriber(
            '~input/class', ClassificationResult, queue_size=1)
        self.subs = [sub_rects, sub_class]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def callback(self, rects_msg, class_msg):

        if len(rects_msg.rects) != len(class_msg.label_proba):
            rospy.logwarn(
                'The sizes of rects and class does not match. Skipping...')
            return

        detections = []

        for index, rect in enumerate(rects_msg.rects):
            if self.target_labels is not None and \
                    class_msg.label_names[index] not in self.target_labels:
                rospy.logwarn('{} is not in target_labels.'.format(class_msg.label_names[index]))
                continue
            detections.append(
                Detection(box=np.array(
                              [rect.x,
                               rect.y,
                               rect.x + rect.width,
                               rect.y + rect.height]),
                          score=class_msg.label_proba[index],
                          class_id=class_msg.labels[index]
                          )
            )

        self.tracker.step(detections=detections)
        active_tracks = self.tracker.active_tracks(min_steps_alive=self.min_steps_alive)

        tracks_msg = TrackArray()
        tracks_msg.header = rects_msg.header
        tracks_msg.tracks = [Track(track_id=(uuid.UUID(active_track.id).int % 2147483647),
                                   rect=Rect(x=int(active_track.box[0]),
                                             y=int(active_track.box[1]),
                                             width=int(active_track.box[2]
                                             - active_track.box[0]),
                                             height=int(active_track.box[3]
                                             - active_track.box[1])
                                             ),
                                   class_id=active_track.class_id,
                                   class_name=self.label_names[active_track.class_id],
                                   score=active_track.score
                                   )
                             for active_track in active_tracks]

        self.pub_tracks.publish(tracks_msg)


if __name__ == '__main__':

    rospy.init_node('motpy_ros_node')
    node = MotpyRectTracker()
    rospy.spin()
