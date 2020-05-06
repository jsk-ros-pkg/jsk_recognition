#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import yaml

import itertools, pkg_resources
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
import numpy as np

try:
    import chainer_mask_rcnn
except ImportError:
    print('''Please install chainer_mask_rcnn:

    sudo pip install chainer-mask-rcnn

''', file=sys.stderr)
    sys.exit(1)

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import MaskRCNNInstanceSegmentationConfig as Config
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import Label
from jsk_recognition_msgs.msg import LabelArray
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_topic_tools import ConnectionBasedTransport
from pcl_msgs.msg import PointIndices
import rospy
from sensor_msgs.msg import Image


class MaskRCNNInstanceSegmentation(ConnectionBasedTransport):

    def __init__(self):
        rospy.logwarn('This node is experimental, and its interface '
                      'can be changed in the future.')

        super(MaskRCNNInstanceSegmentation, self).__init__()
        # gpu
        self.gpu = rospy.get_param('~gpu', 0)
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False

        fg_class_names = rospy.get_param('~fg_class_names')
        if isinstance(fg_class_names, str) and os.path.exists(fg_class_names):
            rospy.loginfo('Loading class names from file: {}'.format(fg_class_names))
            with open(fg_class_names, 'r') as f:
                fg_class_names = yaml.load(f)
        self.fg_class_names = fg_class_names

        pretrained_model = rospy.get_param('~pretrained_model')
        self.classifier_name = rospy.get_param(
            "~classifier_name", rospy.get_name())

        n_fg_class = len(self.fg_class_names)
        self.model = chainer_mask_rcnn.models.MaskRCNNResNet(
            n_layers=50,
            n_fg_class=n_fg_class,
            pretrained_model=pretrained_model,
            anchor_scales=rospy.get_param('~anchor_scales', [4, 8, 16, 32]),
            min_size=rospy.get_param('~min_size', 600),
            max_size=rospy.get_param('~max_size', 1000),
        )
        if self.gpu >= 0:
            self.model.to_gpu(self.gpu)

        self.srv = Server(Config, self.config_callback)

        self.pub_indices = self.advertise(
            '~output/cluster_indices', ClusterPointIndices, queue_size=1)
        self.pub_labels = self.advertise(
            '~output/labels', LabelArray, queue_size=1)
        self.pub_lbl_cls = self.advertise(
            '~output/label_cls', Image, queue_size=1)
        self.pub_lbl_ins = self.advertise(
            '~output/label_ins', Image, queue_size=1)
        self.pub_viz = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pub_rects = self.advertise(
            "~output/rects", RectArray,
            queue_size=1)
        self.pub_class = self.advertise(
            "~output/class", ClassificationResult,
            queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.callback,
                                    queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def config_callback(self, config, level):
        self.model.score_thresh = config.score_thresh
        return config

    def callback(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')
        img_chw = img.transpose(2, 0, 1)  # C, H, W

        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        bboxes, masks, labels, scores = self.model.predict([img_chw])

        bboxes = bboxes[0]
        masks = masks[0]
        labels = labels[0]
        scores = scores[0]

        msg_indices = ClusterPointIndices(header=imgmsg.header)
        msg_labels = LabelArray(header=imgmsg.header)
        # -1: label for background
        lbl_cls = - np.ones(img.shape[:2], dtype=np.int32)
        lbl_ins = - np.ones(img.shape[:2], dtype=np.int32)
        for ins_id, (mask, label) in enumerate(zip(masks, labels)):
            indices = np.where(mask.flatten())[0]
            indices_msg = PointIndices(header=imgmsg.header, indices=indices)
            msg_indices.cluster_indices.append(indices_msg)
            class_name = self.fg_class_names[label]
            msg_labels.labels.append(Label(id=label, name=class_name))
            lbl_cls[mask] = label
            lbl_ins[mask] = ins_id  # instance_id
        self.pub_indices.publish(msg_indices)
        self.pub_labels.publish(msg_labels)

        msg_lbl_cls = bridge.cv2_to_imgmsg(lbl_cls)
        msg_lbl_ins = bridge.cv2_to_imgmsg(lbl_ins)
        msg_lbl_cls.header = msg_lbl_ins.header = imgmsg.header
        self.pub_lbl_cls.publish(msg_lbl_cls)
        self.pub_lbl_ins.publish(msg_lbl_ins)

        cls_msg = ClassificationResult(
            header=imgmsg.header,
            classifier=self.classifier_name,
            target_names=self.fg_class_names,
            labels=labels,
            label_names=[self.fg_class_names[l] for l in labels],
            label_proba=scores,
        )

        rects_msg = RectArray(header=imgmsg.header)
        for bbox in bboxes:
            rect = Rect(x=bbox[1], y=bbox[0],
                        width=bbox[3] - bbox[1],
                        height=bbox[2] - bbox[0])
            rects_msg.rects.append(rect)
        self.pub_rects.publish(rects_msg)
        self.pub_class.publish(cls_msg)

        if self.pub_viz.get_num_connections() > 0:
            n_fg_class = len(self.fg_class_names)
            captions = ['{:d}: {:s}'.format(l, self.fg_class_names[l])
                        for l in labels]
            viz = chainer_mask_rcnn.utils.draw_instance_bboxes(
                img, bboxes, labels + 1, n_class=n_fg_class + 1,
                masks=masks, captions=captions)
            msg_viz = bridge.cv2_to_imgmsg(viz, encoding='rgb8')
            msg_viz.header = imgmsg.header
            self.pub_viz.publish(msg_viz)


if __name__ == '__main__':
    rospy.init_node('mask_rcnn_instance_segmentation')
    node = MaskRCNNInstanceSegmentation()
    rospy.spin()
