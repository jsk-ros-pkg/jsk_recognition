#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
import time
import yaml

from dynamic_reconfigure.server import Server
from jsk_perception.cfg import SSDObjectDetectorConfig as Config

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import ClusterPointIndices
from pcl_msgs.msg import PointIndices

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
   #sys.exit(1)
import chainer
from chainercv.links import SSD300
from chainercv.links import SSD512
from chainercv.visualizations import vis_bbox


chainer.config.cv_resize_backend = 'cv2'


class SSDObjectDetector(ConnectionBasedTransport):

    def __init__(self):
        super(SSDObjectDetector, self).__init__()
        self.gpu = rospy.get_param("~gpu", -1)
        self.classifier_name = rospy.get_param("~classifier_name", rospy.get_name())

        self.cv_bridge = CvBridge()

        # load model
        self.label_names = self.load_label_names()
        rospy.loginfo("Loaded %d labels" % len(self.label_names))

        # model_path: name of pretrained model or path to model file
        model_path = rospy.get_param("~model_path", None)
        model_name = rospy.get_param('~model', 'ssd300')
        if model_name == 'ssd300':
            model_class = SSD300
        elif model_name == 'ssd512':
            model_class = SSD512
        else:
            rospy.logerr('Unsupported ~model: {}'.format(model_name))
        self.model = model_class(
            n_fg_class=len(self.label_names),
            pretrained_model=model_path)
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
            self.model.to_gpu()
        rospy.loginfo("Loaded model: %s" % model_path)

        # dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        # advertise
        self.pub_indices = self.advertise("~output/cluster_indices", ClusterPointIndices,
                                          queue_size=1)
        self.pub_rects = self.advertise("~output/rect", RectArray,
                                        queue_size=1)
        self.pub_class = self.advertise("~output/class", ClassificationResult,
                                        queue_size=1)
        self.pub_image = self.advertise("~output/image", Image,
                                        queue_size=1)

    def subscribe(self):
        self.sub_image = rospy.Subscriber("~input", Image, self.image_cb,
                                          queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub_image.unregister()

    @property
    def visualize(self):
        return self.pub_image.get_num_connections() > 0

    def load_label_names(self):
        label_names = rospy.get_param("~label_names", tuple())
        if not label_names:
            try:
                from chainercv.datasets import voc_detection_label_names
                label_names = voc_detection_label_names
            except:
                from chainercv.datasets import voc_bbox_label_names
                label_names = voc_bbox_label_names
        elif isinstance(label_names, str):
            with open(label_names, "r") as f:
                label_names = tuple(yaml.load(f))
        return label_names

    def config_callback(self, config, level):
        self.model.nms_thresh = config.nms_thresh
        self.model.score_thresh = config.score_thresh
        self.profiling = config.profiling
        return config

    def image_cb(self, msg):
        if self.profiling:
            rospy.loginfo("callback start: incomming msg is %s msec behind" % ((rospy.Time.now() - msg.header.stamp).to_sec() * 1000.0))
        tprev = time.time()
        try:
            # transform image to RGB, float, CHW
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            img = np.asarray(img, dtype=np.float32)
            img = img.transpose(2, 0, 1)  # (H, W, C) -> (C, H, W)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s" % str(e))
            return
        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("convert", (tcur-tprev)*1000))
            tprev = tcur

        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()
        bboxes, labels, scores = self.model.predict([img])
        bboxes, labels, scores = bboxes[0], labels[0], scores[0]

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("predict", (tcur-tprev)*1000))
            tprev = tcur

        cluster_indices_msg = ClusterPointIndices(header=msg.header)
        H = img.shape[1]
        W = img.shape[2]
        for bbox in bboxes:
            ymin = max(0, int(np.floor(bbox[0])))
            xmin = max(0, int(np.floor(bbox[1])))
            ymax = min(H, int(np.ceil(bbox[2])))
            xmax = min(W, int(np.ceil(bbox[3])))
            indices = [range(W*y+xmin, W*y+xmax) for y in range(ymin, ymax)]
            indices = np.array(indices, dtype=np.int32).flatten()
            indices_msg = PointIndices(header=msg.header, indices=indices)
            cluster_indices_msg.cluster_indices.append(indices_msg)

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("make cluster_indices msg", (tcur-tprev)*1000))
            tprev = tcur

        rect_msg = RectArray(header=msg.header)
        for bbox in bboxes:
            rect = Rect(x=bbox[1], y=bbox[0],
                        width= bbox[3] - bbox[1],
                        height=bbox[2] - bbox[0])
            rect_msg.rects.append(rect)

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("make rect msg", (tcur-tprev)*1000))
            tprev = tcur

        cls_msg = ClassificationResult(
            header=msg.header,
            classifier=self.classifier_name,
            target_names=self.label_names,
            labels=labels,
            label_names=[self.label_names[l] for l in labels],
            label_proba=scores,
        )

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("make cls msg", (tcur-tprev)*1000))
            tprev = tcur

        self.pub_indices.publish(cluster_indices_msg)
        self.pub_rects.publish(rect_msg)
        self.pub_class.publish(cls_msg)

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("publish msg", (tcur-tprev)*1000))
            tprev = tcur

        if self.visualize:
            self.publish_bbox_image(img, bboxes, labels, scores, msg.header)

        if self.profiling:
            tcur = time.time()
            rospy.loginfo("%s: elapsed %f msec" % ("callback end", (tcur-tprev)*1000))
            tprev = tcur

    def publish_bbox_image(self, img, bbox, label, score, header):
        vis_bbox(img, bbox, label, score,
                 label_names=self.label_names)
        fig = plt.gcf()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
        fig.clf()
        img.shape = (h, w, 3)
        plt.close()
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(img, "rgb8")
        except Exception as e:
            rospy.logerr("Failed to convert bbox image: %s" % str(e))
            return
        msg.header = header
        self.pub_image.publish(msg)


if __name__ == '__main__':
    rospy.init_node("ssd_object_detector")
    ssd = SSDObjectDetector()
    rospy.spin()
