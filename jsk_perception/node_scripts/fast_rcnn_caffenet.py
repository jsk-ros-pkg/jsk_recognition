#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import cv2
import numpy as np

import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image


if 'FRCN_ROOT' not in os.environ:
    print("Please set 'FRCN_ROOT' environmental variable.")
    sys.exit(1)

FRCN_ROOT = os.environ['FRCN_ROOT']
sys.path.insert(0, os.path.join(FRCN_ROOT, 'caffe-fast-rcnn/python'))
sys.path.insert(0, os.path.join(FRCN_ROOT, 'lib'))

import caffe
from fast_rcnn.test import im_detect
from utils.cython_nms import nms

caffe.set_mode_gpu()
caffe.set_device(0)

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')


def get_obj_proposals(bgr_img):
    import dlib
    rects = []
    rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
    dlib.find_candidate_object_locations(rgb_img, rects, min_size=20)
    proposals = np.zeros((len(rects), 4))
    for i, r in enumerate(rects):
        # xmin ymin xmax ymax
        proposals[i] = [r.left(), r.top(), r.right(), r.bottom()]
    proposals = proposals.astype(np.float32)
    return proposals


def vis_detections(im, class_name, dets, thresh=0.5):
    """Draw detected bounding boxes."""
    inds = np.where(dets[:, -1] >= thresh)[0]
    if len(inds) == 0:
        return im

    out = im.copy()

    for i in inds:
        bbox = dets[i, :4]
        score = dets[i, -1]
        cv2.rectangle(out, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255))
        text = '{:s} {:.3f}'.format(class_name, score)
        cv2.putText(out, text,
                    (int(bbox[0]), int(bbox[1]) - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)

    return out


class FastRCNN(ConnectionBasedTransport):

    def __init__(self, net):
        super(FastRCNN, self).__init__()
        self.net = net
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self._sub = rospy.Subscriber('~input', Image, self._detect)

    def unsubscribe(self):
        self._sub.unregister()

    def _detect(self, msg):
        bridge = cv_bridge.CvBridge()
        im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        out_im = self._detect_obj(im)
        out_msg = bridge.cv2_to_imgmsg(out_im, encoding='bgr8')
        out_msg.header = msg.header
        self._pub.publish(out_msg)

    def _detect_obj(self, im):
        obj_proposals = get_obj_proposals(bgr_img=im)
        rospy.loginfo('{} object proposals'.format(len(obj_proposals)))

        scores, boxes = im_detect(self.net, im, obj_proposals)

        # Visualize detections for each class
        CONF_THRESH = 0.8
        NMS_THRESH = 0.3
        for cls in CLASSES[1:]:
            cls_ind = CLASSES.index(cls)
            cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            keep = np.where(cls_scores >= CONF_THRESH)[0]
            cls_boxes = cls_boxes[keep, :]
            cls_scores = cls_scores[keep]
            dets = np.hstack((cls_boxes,
                            cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, NMS_THRESH)
            dets = dets[keep, :]
            im = vis_detections(im, cls, dets, thresh=CONF_THRESH)
        return im


if __name__ == '__main__':
    prototxt = os.path.join(FRCN_ROOT, 'models/CaffeNet/test.prototxt')
    caffemodel = os.path.join(FRCN_ROOT,
        'data/fast_rcnn_models/caffenet_fast_rcnn_iter_40000.caffemodel')
    caffenet = caffe.Net(prototxt, caffemodel, caffe.TEST)

    rospy.init_node('fast_rcnn_caffenet')
    fast_rcnn = FastRCNN(net=caffenet)
    rospy.spin()
