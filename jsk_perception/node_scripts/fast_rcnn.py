#!/usr/bin/env python

from __future__ import print_function

import os.path as osp
import sys

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
from chainer import cuda
import chainer.serializers as S
from chainer import Variable
import cv2
from distutils.version import LooseVersion
import numpy as np

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import FastRCNNConfig as Config
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult
import jsk_recognition_utils
from jsk_recognition_utils.chainermodels import VGG16FastRCNN
from jsk_recognition_utils.chainermodels import VGG_CNN_M_1024
from jsk_recognition_utils.nms import nms
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospkg
import rospy
from sensor_msgs.msg import Image


def img_preprocessing(orig_img, pixel_means, max_size=1000, scale=600):
    img = orig_img.astype(np.float32, copy=True)
    img -= pixel_means
    im_size_min = np.min(img.shape[0:2])
    im_size_max = np.max(img.shape[0:2])
    im_scale = float(scale) / float(im_size_min)
    if np.rint(im_scale * im_size_max) > max_size:
        im_scale = float(max_size) / float(im_size_max)
    img = cv2.resize(img, None, None, fx=im_scale, fy=im_scale,
                     interpolation=cv2.INTER_LINEAR)
    return img.transpose([2, 0, 1]).astype(np.float32), im_scale


class FastRCNN(ConnectionBasedTransport):

    def __init__(self, model, target_names, pixel_means, use_gpu):
        super(FastRCNN, self).__init__()

        self._srv = Server(Config, self.configCallback)

        self.model = model
        self._pub_rects = self.advertise('~output/rect_array',
                                         RectArray, queue_size=1)
        self._pub_class = self.advertise('~output/class',
                                         ClassificationResult, queue_size=1)
        self.target_names = target_names
        self.pixel_means = np.array(pixel_means, dtype=np.float32)
        self.use_gpu = use_gpu
        self.classifier_name = rospy.get_param("~classifier_name", rospy.get_name())

    def configCallback(self, config, level):
        self.nms_thresh = config.nms_thresh
        self.conf_thresh = config.conf_thresh
        return config

    def subscribe(self):
        self._sub = message_filters.Subscriber('~input', Image)
        self._sub_rects = message_filters.Subscriber('~input/rect_array',
                                                     RectArray)
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 100)
        subs = [self._sub, self._sub_rects]
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(subs, queue_size)
        sync.registerCallback(self._detect)

    def unsubscribe(self):
        self._sub.unregister()
        self._sub_rects.unregister()

    def _detect(self, imgmsg, rects_msg):
        bridge = cv_bridge.CvBridge()
        im_orig = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
        im, im_scale = img_preprocessing(im_orig, self.pixel_means)
        rects_orig = jsk_recognition_utils.rects_msg_to_ndarray(rects_msg)
        if len(rects_orig) == 0:
            return
        rects = rects_orig * im_scale
        scores, bbox_pred = self._im_detect(im, rects)

        rects = RectArray(header=imgmsg.header)
        labels = []
        label_proba = []
        for cls_id in range(1, len(self.target_names)):
            _cls = scores[:, cls_id][:, np.newaxis]
            _bbx = bbox_pred[:, cls_id * 4: (cls_id + 1) * 4]
            dets = np.hstack((_bbx, _cls))
            keep = nms(dets, self.nms_thresh)
            dets = dets[keep, :]
            orig_rects = cuda.cupy.asnumpy(rects_orig)[keep, :]

            inds = np.where(dets[:, -1] >= self.conf_thresh)[0]

            for i in inds:
                _bbox = dets[i, :4]
                x1, y1, x2, y2 = orig_rects[i]
                width = x2 - x1
                height = y2 - y1
                center_x = x1 + 0.5 * width
                center_y = y1 + 0.5 * height

                dx, dy, dw, dh = _bbox
                _center_x = dx * width + center_x
                _center_y = dy * height + center_y
                _width = np.exp(dw) * width
                _height = np.exp(dh) * height

                x1 = _center_x - 0.5 * _width
                y1 = _center_y - 0.5 * _height
                x2 = _center_x + 0.5 * _width
                y2 = _center_y + 0.5 * _height
                rect = Rect(x=x1, y=y1, width=x2-x1, height=y2-y1)
                rects.rects.append(rect)
                labels.append(cls_id)
                label_proba.append(dets[:, -1][i])

        # publish classification result
        clss = ClassificationResult(
            header=imgmsg.header,
            classifier=self.classifier_name,
            target_names=self.target_names,
            labels=labels,
            label_names=[self.target_names[l] for l in labels],
            label_proba=label_proba,
        )
        self._pub_rects.publish(rects)
        self._pub_class.publish(clss)

    def _im_detect(self, im, rects):
        xp = cuda.cupy if self.use_gpu else np
        im = xp.asarray(im)
        rects = xp.asarray(rects)
        x_data = im[xp.newaxis, :, :, :]
        # batch_indices is always 0 when batch size is 1
        batch_indices = xp.zeros((len(rects), 1), dtype=np.float32)
        rects = xp.hstack((batch_indices, rects))
        if LooseVersion(chainer.__version__).version[0] < 2:
            x = Variable(x_data, volatile=True)
            rects_val = Variable(rects, volatile=True)
            self.model.train = False
            cls_score, bbox_pred = self.model(x, rects_val)
        else:
            with chainer.using_config('train', False), \
                 chainer.no_backprop_mode():
                x = Variable(x_data)
                rects_val = Variable(rects)
                cls_score, bbox_pred = self.model(x, rects_val)

        scores = cuda.to_cpu(cls_score.data)
        bbox_pred = cuda.to_cpu(bbox_pred.data)
        return scores, bbox_pred


def main():
    rospy.init_node('fast_rcnn_caffenet')

    # get parameters
    try:
        model_name = rospy.get_param('~model')
    except KeyError as e:
        rospy.logerr('Unspecified rosparam: {0}'.format(e))
        sys.exit(1)

    gpu = rospy.get_param('~gpu', -1)
    use_gpu = True if gpu >= 0 else False

    # setup model
    PKG = 'jsk_perception'
    rp = rospkg.RosPack()
    data_path = osp.join(rp.get_path(PKG), 'trained_data')
    if model_name == 'vgg_cnn_m_1024':
        model = VGG_CNN_M_1024()
        chainermodel = osp.join(data_path, 'vgg_cnn_m_1024.chainermodel')
    elif model_name == 'vgg16':
        model = VGG16FastRCNN()
        chainermodel = osp.join(data_path, 'vgg16_fast_rcnn.chainermodel')
    else:
        rospy.logerr('Unsupported model: {0}'.format(model_name))
        sys.exit(1)
    rospy.loginfo('Loading chainermodel')
    S.load_hdf5(chainermodel, model)
    if use_gpu:
        model.to_gpu(gpu)
    rospy.loginfo('Finished loading chainermodel')

    # assumptions
    target_names = [
        '__background__',
        'aeroplane',
        'bicycle',
        'bird',
        'boat',
        'bottle',
        'bus',
        'car',
        'cat',
        'chair',
        'cow',
        'diningtable',
        'dog',
        'horse',
        'motorbike',
        'person',
        'pottedplant',
        'sheep',
        'sofa',
        'train',
        'tvmonitor',
    ]
    pixel_means = [102.9801, 115.9465, 122.7717]

    fast_rcnn = FastRCNN(
        model=model, target_names=target_names,
        pixel_means=pixel_means, use_gpu=use_gpu)
    rospy.spin()


if __name__ == '__main__':
    main()
