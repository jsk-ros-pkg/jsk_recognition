#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division

from collections import OrderedDict

import cv2
import cv_bridge
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
from pcl_msgs.msg import PointIndices
import rospy
from sensor_msgs.msg import Image
import torch
from torch.autograd import Variable

from jsk_perception.cfg import CRAFTConfig as Config

import craft.craft as craft
import craft.craft_utils as craft_utils
import craft.imgproc as imgproc
from craft.refinenet import RefineNet


def copy_state_dict(state_dict):
    if list(state_dict.keys())[0].startswith("module"):
        start_idx = 1
    else:
        start_idx = 0
    new_state_dict = OrderedDict()
    for k, v in state_dict.items():
        name = ".".join(k.split(".")[start_idx:])
        new_state_dict[name] = v
    return new_state_dict


def test_net(net, image, text_threshold, link_threshold, text_low_bound_score,
             device, poly=False, refine_net=None,
             mag_ratio=1.5,
             max_image_size=1280):
    # resize
    img_resized, target_ratio, size_heatmap = imgproc.resize_aspect_ratio(
        image,
        max_image_size,
        interpolation=cv2.INTER_LINEAR,
        mag_ratio=mag_ratio)
    ratio_h = ratio_w = 1 / target_ratio

    # preprocessing
    x = imgproc.normalizeMeanVariance(img_resized)
    # [h, w, c] to [c, h, w]
    x = torch.from_numpy(x).permute(2, 0, 1)
    # [c, h, w] to [b, c, h, w]
    x = Variable(x.unsqueeze(0))
    x = x.to(device)

    # forward pass
    with torch.no_grad():
        y, feature = net(x)

    # make score and link map
    score_text = y[0, :, :, 0].cpu().data.numpy()
    score_link = y[0, :, :, 1].cpu().data.numpy()

    # refine link
    if refine_net is not None:
        with torch.no_grad():
            y_refiner = refine_net(y, feature)
        score_link = y_refiner[0, :, :, 0].cpu().data.numpy()

    # Post-processing
    boxes, polys = craft_utils.getDetBoxes(
        score_text, score_link,
        text_threshold, link_threshold,
        text_low_bound_score, poly)

    # coordinate adjustment
    boxes = craft_utils.adjustResultCoordinates(boxes, ratio_w, ratio_h)
    polys = craft_utils.adjustResultCoordinates(polys, ratio_w, ratio_h)
    for k in range(len(polys)):
        if polys[k] is None:
            polys[k] = boxes[k]

    render_img = score_text.copy()
    render_img = np.hstack((render_img, score_link))
    ret_score_text = imgproc.cvt2HeatmapImg(render_img)
    return boxes, polys, ret_score_text


class CRAFTNode(ConnectionBasedTransport):

    def __init__(self):
        super(CRAFTNode, self).__init__()

        # init neural networks.
        net = craft.CRAFT()
        model_path = rospy.get_param('~model_path')
        refine_model_path = rospy.get_param('~refine_model_path')

        gpu = rospy.get_param('~gpu', -1)
        if torch.cuda.is_available() and gpu >= 0:
            device = torch.device('cuda:{}'.format(gpu))
        else:
            device = torch.device('cpu')
        net.load_state_dict(
            copy_state_dict(torch.load(model_path, map_location=device)))

        net = net.to(device)
        net.eval()

        refine_net = RefineNet()
        refine_net.load_state_dict(
            copy_state_dict(
                torch.load(refine_model_path, map_location=device)))
        refine_net = refine_net.to(device)
        if torch.cuda.is_available() and gpu >= 0:
            refine_net = torch.nn.DataParallel(refine_net)
        refine_net.eval()

        self.net = net
        self.refine_net = refine_net
        self.device = device

        # dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        # publish topics
        self.pub_polygons = self.advertise(
            "~output/polygons", PolygonArray,
            queue_size=1)
        self.pub_rects = self.advertise(
            "~output/rects", RectArray,
            queue_size=1)
        self.pub_indices = self.advertise(
            '~output/cluster_indices', ClusterPointIndices, queue_size=1)

    def config_callback(self, config, level):
        self.text_threshold = config.text_threshold
        self.link_threshold = config.link_threshold
        self.text_low_bound_score = config.text_low_bound_score
        self.mag_ratio = config.mag_ratio
        self.max_image_size = config.max_image_size
        return config

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', Image, self.callback, queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')

        bboxes, polys, score_text = test_net(
            self.net, img,
            self.text_threshold,
            self.link_threshold,
            self.text_low_bound_score,
            self.device, False,
            self.refine_net,
            mag_ratio=self.mag_ratio,
            max_image_size=self.max_image_size)

        msg_indices = ClusterPointIndices(header=img_msg.header)
        polygon_array_msg = PolygonArray(header=img_msg.header)
        for poly in polys:
            indices_img = np.zeros(
                (img.shape[0], img.shape[1]), dtype=np.uint8)
            poly = np.array(poly).astype(np.int32).reshape((-1))
            poly = poly.reshape(-1, 2)
            cv2.fillPoly(
                indices_img, [poly.reshape((-1, 1, 2))], color=255)
            indices = np.where(indices_img.reshape(-1))[0]
            indices_msg = PointIndices(
                header=img_msg.header, indices=indices)
            msg_indices.cluster_indices.append(indices_msg)

            polygon_stamp_msg = PolygonStamped(header=img_msg.header)
            for x, y in poly:
                polygon_stamp_msg.polygon.points.append(Point32(x=x, y=y))
            polygon_array_msg.polygons.append(polygon_stamp_msg)

        rect_array_msg = RectArray(header=img_msg.header)
        for poly in polys:
            poly = np.array(poly, dtype=np.int32)
            x_min = max(np.min(poly[:, 0]), 0)
            x_max = min(np.max(poly[:, 0]), img.shape[1])
            y_min = max(np.min(poly[:, 1]), 0)
            y_max = min(np.max(poly[:, 1]), img.shape[0])
            width = x_max - x_min
            height = y_max - y_min
            rect_array_msg.rects.append(
                Rect(x=x_min, y=y_min, width=width, height=height))
        self.pub_polygons.publish(polygon_array_msg)
        self.pub_rects.publish(rect_array_msg)
        self.pub_indices.publish(msg_indices)


if __name__ == '__main__':
    rospy.init_node('craft_node')
    node = CRAFTNode()  # NOQA
    rospy.spin()
