#!/usr/bin/env python

import os.path as osp
import sys

from chainer import cuda
import chainer.serializers as S
from chainer import Variable
import cv2
import numpy as np

import cv_bridge
from jsk_recognition_msgs.msg import RectArray
import jsk_recognition_utils
from jsk_recognition_utils.chainermodels import VGG16FastRCNN
from jsk_recognition_utils.chainermodels import VGG_CNN_M_1024
from jsk_recognition_utils.nms import nms
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import jsk_logfatal
from jsk_topic_tools.log_utils import jsk_loginfo
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
        self.model = model
        self._pub = self.advertise('~output', Image, queue_size=1)
        self.target_names = target_names
        self.pixel_means = np.array(pixel_means, dtype=np.float32)
        self.use_gpu = use_gpu

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
        out_im = self._draw_result(
            im_orig, scores, bbox_pred, rects_orig, nms_thresh=0.3, conf=0.8)
        out_msg = bridge.cv2_to_imgmsg(out_im, encoding='bgr8')
        out_msg.header = imgmsg.header
        self._pub.publish(out_msg)

    def _draw_result(self, im, clss, bbox, rects, nms_thresh, conf):
        for cls_id in range(1, len(self.target_names)):
            _cls = clss[:, cls_id][:, np.newaxis]
            _bbx = bbox[:, cls_id * 4: (cls_id + 1) * 4]
            dets = np.hstack((_bbx, _cls))
            keep = nms(dets, nms_thresh)
            dets = dets[keep, :]
            orig_rects = cuda.cupy.asnumpy(rects)[keep, :]

            inds = np.where(dets[:, -1] >= conf)[0]
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

                cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)),
                              (0, 0, 255), 2, cv2.CV_AA)
                ret, baseline = cv2.getTextSize(
                    self.target_names[cls_id], cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, 1)
                cv2.rectangle(im, (int(x1), int(y2) - ret[1] - baseline),
                              (int(x1) + ret[0], int(y2)), (0, 0, 255), -1)
                cv2.putText(im, self.target_names[cls_id],
                            (int(x1), int(y2) - baseline),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (255, 255, 255), 1, cv2.CV_AA)
        return im

    def _im_detect(self, im, rects):
        xp = cuda.cupy if self.use_gpu else np
        im = xp.asarray(im)
        rects = xp.asarray(rects)
        x_data = im[xp.newaxis, :, :, :]
        # batch_indices is always 0 when batch size is 1
        batch_indices = xp.zeros((len(rects), 1), dtype=np.float32)
        rects = xp.hstack((batch_indices, rects))
        x = Variable(x_data, volatile=True)
        rects_val = Variable(rects, volatile=True)
        self.model.train = False
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

    # FIXME: In CPU mode, there is no detections.
    if not cuda.available:
        jsk_logfatal('CUDA environment is required.')
        sys.exit(1)
    use_gpu = True

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
    jsk_loginfo('Loading chainermodel')
    S.load_hdf5(chainermodel, model)
    if use_gpu:
        model.to_gpu()
    jsk_loginfo('Finished loading chainermodel')

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
