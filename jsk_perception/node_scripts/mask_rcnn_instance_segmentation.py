#!/usr/bin/env python

from __future__ import print_function
import sys

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
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import Label
from jsk_recognition_msgs.msg import LabelArray
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
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        chainer.global_config.train = False
        chainer.global_config.enable_backprop = False

        self.fg_class_names = rospy.get_param('~fg_class_names')
        pretrained_model = rospy.get_param('~pretrained_model')

        n_fg_class = len(self.fg_class_names)
        self.model = chainer_mask_rcnn.models.MaskRCNNResNet(
            n_layers=50,
            n_fg_class=n_fg_class,
            pretrained_model=pretrained_model,
        )
        self.model.score_thresh = rospy.get_param('~score_thresh', 0.7)
        if self.gpu >= 0:
            self.model.to_gpu()

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

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.callback,
                                    queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, imgmsg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='rgb8')
        img_chw = img.transpose(2, 0, 1)  # C, H, W

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
