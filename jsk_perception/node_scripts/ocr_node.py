#!/usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division
from __future__ import print_function

import multiprocessing
from multiprocessing.pool import ThreadPool as Pool
import os
import os.path as osp

import std_msgs.msg
import cv2
import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_recognition_msgs.msg import Label
from jsk_recognition_msgs.msg import LabelArray
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_utils.put_text import put_text_to_image
from jsk_recognition_utils import get_tile_image
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import numpy as np
import pytesseract
import rospy
from sensor_msgs.msg import Image

from jsk_perception.cfg import OCRConfig as Config


def crop_img(img, poly):
    if poly.shape != (4, 2):
        raise ValueError('Not supported shape size {}'.format(poly.shape))
    # make clock-wise order
    poly = np.array(poly, dtype=np.int32)
    startidx = poly.sum(axis=1).argmin()
    poly = np.roll(poly, len(poly) - startidx, 0)
    # crop target area.
    poly = np.array(poly, dtype=np.int32)
    x_min = max(np.min(poly[:, 0]), 0)
    x_max = min(np.max(poly[:, 0]), img.shape[1])
    y_min = max(np.min(poly[:, 1]), 0)
    y_max = min(np.max(poly[:, 1]), img.shape[0])
    w = x_max - x_min
    h = y_max - y_min
    pts1 = np.float32(poly)
    pts2 = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    rot_mat = cv2.getPerspectiveTransform(pts1, pts2)
    croppped_img = cv2.warpPerspective(img, rot_mat, (w, h))
    return croppped_img


def ocr_image(process_index, poly, img, lang='eng'):
    croppped_img = crop_img(img, poly)
    if croppped_img.shape[0] == 0 or croppped_img.shape[1] == 0:
        txt = u''
        binary_img = np.zeros((0, 0), dtype=np.uint8)
    else:
        gray = cv2.cvtColor(croppped_img, cv2.COLOR_RGB2GRAY)
        _, binary_img = cv2.threshold(
            gray, 127, 255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        txt = pytesseract.image_to_string(
            binary_img, lang=lang, config="--psm 6")
        txt = txt.encode('utf-8')
        txt = txt.lstrip().rstrip()
    return process_index, txt, croppped_img, binary_img


def visualize_polygons_with_texts(
        img, polys, texts,
        font_path,
        box_thickness=3,
        font_size=16):
    if not os.path.exists(font_path):
        raise OSError("Font not exists!")
    img = np.array(img)
    if texts is None:
        texts = [''] * len(polys)

    # draw polygons
    for poly in polys:
        poly = np.array(poly).astype(np.int32).reshape((-1))
        poly = poly.reshape(-1, 2)
        cv2.polylines(
            img, [poly.reshape((-1, 1, 2))],
            True, color=(255, 0, 0), thickness=box_thickness)

    # draw texts
    for poly, text in zip(polys, texts):
        poly = np.array(poly).astype(np.int32).reshape((-1))
        poly = poly.reshape(-1, 2)
        if isinstance(text, str):
            text = "{}".format(text)
        else:
            text = "{}".format(text.decode('utf-8'))
        pos = (poly[0][0] + 1, poly[0][1] + 1)
        put_text_to_image(
            img, text, pos,
            font_path, font_size,
            color=(0, 0, 255),
            background_color=(255, 255, 255),
            loc='center')
    return img


class OCRNode(ConnectionBasedTransport):

    def __init__(self):
        super(OCRNode, self).__init__()
        self.bridge = cv_bridge.CvBridge()

        self.valid_font = False
        self.subscribe_polygon = Config.defaults['subscribe_polygon']
        self.approximate_sync = Config.defaults['approximate_sync']
        self.queue_size = Config.defaults['queue_size']

        # dynamic reconfigure
        self.srv = Server(Config, self.config_callback)

        # publish topics
        self.pub_str = self.advertise(
            '~output', std_msgs.msg.String, queue_size=1)
        self.pub_viz = self.advertise(
            '~output/viz', Image, queue_size=1)
        self.pub_labels = self.advertise(
            '~output/labels', LabelArray, queue_size=1)
        self.pub_debug_viz = self.advertise(
            '~output/debug/viz', Image, queue_size=1)
        self.pub_debug_binary_viz = self.advertise(
            '~output/debug/binary_viz', Image, queue_size=1)

    def config_callback(self, config, level):
        resubscribe = False
        if self.subscribe_polygon != config.subscribe_polygon \
           or self.approximate_sync != config.approximate_sync \
           or self.queue_size != config.queue_size:
            resubscribe = True

        self.language = config.language
        self.font_size = config.font_size
        self.font_path = config.font_path
        self.valid_font = osp.exists(self.font_path)
        if self.valid_font is False:
            rospy.logwarn('Not valid font_path: {}'.format(self.font_path))
        self.box_thickness = config.box_thickness
        self.subscribe_polygon = config.subscribe_polygon

        self.n_jobs = config.number_of_jobs
        if self.n_jobs == -1:
            self.n_jobs = multiprocessing.cpu_count()
        self.n_jobs = max(self.n_jobs, 1)

        self.resolution_factor = config.resolution_factor
        self.interpolation_method = config.interpolation_method

        if resubscribe and self.is_subscribed():
            self.unsubscribe()
            self.subscribe()
        return config

    def subscribe(self):
        sub_image = message_filters.Subscriber('~input', Image)
        subs = [sub_image]
        if self.subscribe_polygon:
            sub_polygons = message_filters.Subscriber(
                '~input/polygons', PolygonArray)
            subs.append(sub_polygons)
            callback = self.polygons_callback
        else:
            sub_rects = message_filters.Subscriber('~input/rects', RectArray)
            subs.append(sub_rects)
            callback = self.rects_callback

        if self.approximate_sync:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                subs,
                queue_size=self.queue_size, slop=slop)
            sync.registerCallback(callback)
        else:
            sync = message_filters.TimeSynchronizer(
                subs, queue_size=self.queue_size)
            sync.registerCallback(callback)
        self.subs = subs

    def unsubscribe(self):
        for sub in self.subs:
            sub.sub.unregister()

    def publish_results(self, img, polys, texts, header, imgs=None,
                        binary_imgs=None):
        label_array_msg = LabelArray(header=header)
        for i, text in enumerate(texts):
            label_array_msg.labels.append(
                Label(id=i, name=text.decode('utf-8')))
        self.pub_labels.publish(label_array_msg)

        if self.pub_viz.get_num_connections() > 0:
            img = cv2.resize(
                img, None,
                fx=self.resolution_factor,
                fy=self.resolution_factor,
                interpolation=self.interpolation_method)
            if self.valid_font:
                viz = visualize_polygons_with_texts(
                    img, polys * self.resolution_factor,
                    texts=texts,
                    font_path=self.font_path,
                    box_thickness=self.box_thickness,
                    font_size=self.font_size)
            msg_viz = self.bridge.cv2_to_imgmsg(viz, encoding='rgb8')
            msg_viz.header = header
            self.pub_viz.publish(msg_viz)

        if self.pub_debug_viz.get_num_connections() > 0:
            if not imgs:
                return
            viz = get_tile_image(imgs)
            msg_viz = self.bridge.cv2_to_imgmsg(viz, encoding='rgb8')
            msg_viz.header = header
            self.pub_debug_viz.publish(msg_viz)

        if self.pub_debug_binary_viz.get_num_connections() > 0:
            if not binary_imgs:
                return
            viz = get_tile_image(binary_imgs)
            msg_viz = self.bridge.cv2_to_imgmsg(viz, encoding='mono8')
            msg_viz.header = header
            self.pub_debug_binary_viz.publish(msg_viz)

        # Sort the polygons in order of distance from the top left.
        if len(polys) > 0:
            polys = np.array(polys, dtype=np.int32)
            indices = np.argsort(polys.sum(axis=2).min(axis=1))
            text = ' '.join([texts[i].decode('utf-8') for i in indices])
        else:
            text = ''
        self.pub_str.publish(
            std_msgs.msg.String(data=text))

    def rects_callback(self, img_msg, rects_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        polys = []
        for rect in rects_msg.rects:
            polys.append(
                np.array([[rect.x, rect.y],
                          [rect.x, rect.y + rect.height],
                          [rect.x + rect.width, rect.y + rect.height],
                          [rect.x + rect.width, rect.y]], dtype=np.int32))
        polys = np.array(polys)
        texts, imgs, binary_imgs = self.process_ocr(img, polys)
        self.publish_results(img, polys, texts, img_msg.header)

    def polygons_callback(self, img_msg, polygons_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        polys = []
        for polygon_stamp_msg in polygons_msg.polygons:
            polys.append(
                np.array([[point.x, point.y]
                          for point in polygon_stamp_msg.polygon.points]))
        polys = np.array(polys, dtype=np.int32)
        texts, imgs, binary_imgs = self.process_ocr(img, polys)
        self.publish_results(img, polys, texts, img_msg.header,
                             imgs=imgs, binary_imgs=binary_imgs)

    def process_ocr(self, img, polys):
        texts = []
        imgs = []
        binary_imgs = []
        if len(polys) > 0:
            n_jobs = min(self.n_jobs, len(polys))
            process = Pool(n_jobs)
            multiple_results = [
                process.apply_async(ocr_image, (i, poly, img, self.language))
                for i, poly in enumerate(polys)]
            process.close()
            process.join()
            results = sorted([res.get() for res in multiple_results], key=lambda a: a[0])
            texts = [b for a, b, c, d in results]
            imgs = []
            for _, _, img, _ in results:
                if img.shape[0] > 0 and img.shape[1] > 0:
                    imgs.append(img.copy())
            binary_imgs = []
            for _, _, _, img in results:
                if img.shape[0] > 0 and img.shape[1] > 0:
                    binary_imgs.append(img.copy())
        return texts, imgs, binary_imgs


if __name__ == '__main__':
    rospy.init_node('ocr_node')
    ocr = OCRNode()  # NOQA
    rospy.spin()
