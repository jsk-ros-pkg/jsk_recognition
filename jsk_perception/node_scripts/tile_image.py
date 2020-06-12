#!/usr/bin/env python
# -*- coding: utf-8 -*-

import collections
import sys
import cv2
import cv_bridge
import jsk_recognition_utils
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy
from sensor_msgs.msg import Image
import itertools, pkg_resources
from distutils.version import StrictVersion
from threading import Lock


def draw_text_box(img, text, font_scale=0.8, thickness=2,
                  color=(0, 255, 0), fg_color=(0, 0, 0), loc='ltb'):
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    size, baseline = cv2.getTextSize(text, font_face, font_scale, thickness)

    H, W = img.shape[:2]

    if loc == 'ltb':  # left + top + below
        # pt: (x, y)
        pt1 = (0, 0)
        pt2 = (size[0], size[1] + baseline)
        pt3 = (0, size[1])
    elif loc == 'rba':  # right + bottom + above
        pt1 = (W - size[0], H - size[1] - baseline)
        pt2 = (W, H)
        pt3 = (W - size[0], H - baseline)
    else:
        raise ValueError
    if color is not None:
        cv2.rectangle(img, pt1, pt2, color=color, thickness=-1)
    cv2.putText(img, text, pt3, font_face, font_scale, fg_color, thickness)


class TileImages(ConnectionBasedTransport):
    def __init__(self):
        super(TileImages, self).__init__()
        self.lock = Lock()
        self.input_topics = rospy.get_param('~input_topics', [])
        if not self.input_topics:
            rospy.logerr('need to specify input_topics')
            sys.exit(1)
        self._shape = rospy.get_param('~shape', None)
        if self._shape:
            if not (isinstance(self._shape, collections.Sequence) and
                    len(self._shape) == 2):
                rospy.logerr('~shape must be a list of 2 float values.')
                sys.exit(1)
            if (self._shape[0] * self._shape[1]) < len(self.input_topics):
                rospy.logerr('Tile size must be larger than # of input topics')
                sys.exit(1)
        self.cache_img = None
        self.draw_topic_name = rospy.get_param('~draw_topic_name', False)
        self.approximate_sync = rospy.get_param('~approximate_sync', True)
        self.no_sync = rospy.get_param('~no_sync', False)
        self.font_scale = rospy.get_param('~font_scale', 0.8)
        if (not self.no_sync and
            StrictVersion(pkg_resources.get_distribution('message_filters').version) < StrictVersion('1.11.4') and
            self.approximate_sync):
            rospy.logerr('hydro message_filters does not support approximate sync. Force to set ~approximate_sync=false')
            self.approximate_sync = False
        self.pub_img = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub_img_list = []
        if self.no_sync:
            self.input_imgs = {}
            self.sub_img_list = [rospy.Subscriber(topic, Image, self.simple_callback(topic), queue_size=1) for topic in self.input_topics]
            rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        else:
            queue_size = rospy.get_param('~queue_size', 10)
            slop = rospy.get_param('~slop', 1)
            for i, input_topic in enumerate(self.input_topics):
                sub_img = message_filters.Subscriber(input_topic, Image)
                self.sub_img_list.append(sub_img)
            if self.approximate_sync:
                sync = message_filters.ApproximateTimeSynchronizer(
                    self.sub_img_list, queue_size=queue_size, slop=slop)
                sync.registerCallback(self._apply)
            else:
                sync = message_filters.TimeSynchronizer(
                    self.sub_img_list, queue_size=queue_size)
                sync.registerCallback(self._apply)
    def unsubscribe(self):
        for sub in self.sub_img_list:
            sub.sub.unregister()
    def timer_callback(self, event):
        with self.lock:
            imgs = [self.input_imgs[topic] for topic in self.input_topics
                    if topic in self.input_imgs]
            self._append_images(imgs)
    def simple_callback(self, target_topic):
        def callback(msg):
            with self.lock:
                bridge = cv_bridge.CvBridge()
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.input_imgs[target_topic] = img
                if self.draw_topic_name:
                    draw_text_box(img, rospy.resolve_name(target_topic),
                                  font_scale=self.font_scale)
        return callback
    def _append_images(self, imgs):
        if not imgs:
            return
        # convert tile shape: (Y, X) -> (X, Y)
        # if None, shape is automatically decided to be square AMAP.
        shape_xy = self._shape[::-1] if self._shape else None
        if self.cache_img is None:
            out_bgr = jsk_recognition_utils.get_tile_image(
                imgs, tile_shape=shape_xy)
            self.cache_img = out_bgr
        else:
            try:
                out_bgr = jsk_recognition_utils.get_tile_image(
                    imgs, tile_shape=shape_xy, result_img=self.cache_img)
            except ValueError:  # cache miss
                out_bgr = jsk_recognition_utils.get_tile_image(
                    imgs, tile_shape=shape_xy)
                self.cache_img = out_bgr
        bridge = cv_bridge.CvBridge()
        imgmsg = bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
        self.pub_img.publish(imgmsg)
    def _apply(self, *msgs):
        bridge = cv_bridge.CvBridge()
        imgs = []
        for msg, topic in zip(msgs, self.input_topics):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.draw_topic_name:
                draw_text_box(img, rospy.resolve_name(topic),
                              font_scale=self.font_scale)
            imgs.append(img)
        self._append_images(imgs)


if __name__ == '__main__':
    rospy.init_node('tile_image')
    tile_image = TileImages()
    rospy.spin()
