#!/usr/bin/env python3

try:
    from motpy import Detection, MultiObjectTracker
    from jsk_recognition_utils.panorama_multi_object_tracker import PanoramaMultiObjectTracker
except ModuleNotFoundError:
    print('motpy is not found. Please install it.')
    import sys
    sys.exit(1)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Track
from jsk_recognition_msgs.msg import TrackArray

from jsk_recognition_utils.panorama_utils import load_label_names
from jsk_recognition_utils.panorama_utils import visualize_tracks

import uuid


class MotpyROS(object):

    def __init__(self):

        super(MotpyROS, self).__init__()

        self.bridge = CvBridge()

        self.panorama_mode = rospy.get_param('~panorama_mode', False)
        dt = rospy.get_param('~dt')

        min_iou = rospy.get_param('~min_iou', 0.1)
        multi_match_min_iou = rospy.get_param('~multi_match_min_iou', 1.000001)
        kwargs = {'min_iou': min_iou,
                  'multi_match_min_iou': multi_match_min_iou}

        if self.panorama_mode:
            msg_image = rospy.wait_for_message('~input', Image)
            self.width_image = msg_image.width
            self.tracker = PanoramaMultiObjectTracker(
                                dt=dt,
                                image_width=self.width_image,
                                matching_fn_kwargs=kwargs)
        else:
            self.tracker = MultiObjectTracker(
                                dt=dt,
                                matching_fn_kwargs=kwargs)

        self.target_labels = rospy.get_param('~target_labels', None)
        self.label_names = load_label_names()

        self.pub_visualized_image = rospy.Publisher(
            '~output/viz', Image, queue_size=1)
        self.pub_tracks = rospy.Publisher(
            '~output/tracks', TrackArray, queue_size=1)

        self.subscribe()

    def subscribe(self):

        queue_size = rospy.get_param('~queue_size', 100)
        sub_img = message_filters.Subscriber(
            '~input', Image, queue_size=1)
        sub_rects = message_filters.Subscriber(
            '~input/rects', RectArray, queue_size=1)
        sub_class = message_filters.Subscriber(
            '~input/class', ClassificationResult, queue_size=1)
        self.subs = [sub_img, sub_rects, sub_class]
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.callback)

    def callback(self, img_msg, rects_msg, class_msg):

        input_frame = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8')

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
                Detection(box=[rect.x,
                               rect.y,
                               rect.x + rect.width,
                               rect.y + rect.height],
                          score=class_msg.label_proba[index],
                          class_id=class_msg.labels[index]
                          )
            )

        self.tracker.step(detections=detections)
        active_tracks = self.tracker.active_tracks(min_steps_alive=1)

        tracks_msg = TrackArray()
        tracks_msg.header = img_msg.header
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

        if self.pub_tracks.get_num_connections() > 0:
            self.pub_tracks.publish(tracks_msg)

        if self.pub_visualized_image.get_num_connections() > 0:
            visualized_frame = visualize_tracks(input_frame, tracks_msg)
            msg = self.bridge.cv2_to_imgmsg(visualized_frame, "bgr8")
            msg.header = img_msg.header
            self.pub_visualized_image.publish(msg)


if __name__ == '__main__':

    rospy.init_node('motpy_ros_node')
    node = MotpyROS()
    rospy.spin()
