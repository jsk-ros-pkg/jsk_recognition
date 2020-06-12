#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import rospy
import itertools, pkg_resources, sys
from distutils.version import LooseVersion
if LooseVersion(pkg_resources.get_distribution("chainer").version) >= LooseVersion('7.0.0') and \
   sys.version_info.major == 2:
   print('''Please install chainer <= 7.0.0:

    sudo pip install chainer==6.7.0

c.f https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485
''', file=sys.stderr)
   sys.exit(1)
if [p for p in list(itertools.chain(*[pkg_resources.find_distributions(_) for _ in sys.path])) if "cupy-" in p.project_name ] == []:
   print('''Please install CuPy

    sudo pip install cupy-cuda[your cuda version]
i.e.
    sudo pip install cupy-cuda91

''', file=sys.stderr)
   sys.exit(1)
import chainer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Label
from jsk_recognition_msgs.msg import LabelArray

from deep_sort.deep_sort_tracker import DeepSortTracker


class DeepSortTrackerNode(object):

    def __init__(self):
        super(DeepSortTrackerNode, self).__init__()
        self.bridge = CvBridge()

        self.target_labels = rospy.get_param('~target_labels', None)
        self.gpu = rospy.get_param('~gpu', 0)
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False

        pretrained_model = rospy.get_param('~pretrained_model')
        self.tracker = DeepSortTracker(gpu=self.gpu,
                                       pretrained_model=pretrained_model)
        self.image_pub = rospy.Publisher(
            '~output/viz',
            Image, queue_size=1)
        self.pub_labels = rospy.Publisher(
            '~output/labels', LabelArray, queue_size=1)

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
        bridge = self.bridge
        tracker = self.tracker

        frame = bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8')

        scores = []
        rects = []
        for i, r in enumerate(rects_msg.rects):
            if self.target_labels is not None and \
               class_msg.label_names[i] not in self.target_labels:
                continue
            rects.append((int(r.x), int(r.y),
                          int(r.width),
                          int(r.height)))
            scores.append(class_msg.label_proba[i])
        rects = np.array(rects, 'f')
        scores = np.array(scores, 'f')

        if len(rects) > 0:
            tracker.track(frame, rects, scores)

        if self.pub_labels.get_num_connections() > 0:
            msg_labels = LabelArray(header=img_msg.header)
            for object_id, target_object in tracker.tracking_objects.items():
                if target_object['out_of_frame']:
                    continue
                msg_labels.labels.append(Label(id=object_id))
            self.pub_labels.publish(msg_labels)

        if self.image_pub.get_num_connections() > 0:
            frame = tracker.visualize(frame, rects)
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header = img_msg.header
            self.image_pub.publish(msg)


def main():
    rospy.init_node('deep_sort_tracker_node')
    dstn = DeepSortTrackerNode()  # NOQA
    rospy.spin()


if __name__ == '__main__':
    main()
