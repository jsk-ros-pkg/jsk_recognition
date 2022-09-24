#!/usr/bin/env python

import os.path as osp

import cv2
from jsk_recognition_utils import cv_bridge
import dynamic_reconfigure.server
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_utils.color import labelcolormap
from jsk_recognition_utils.put_text import put_text_to_image
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools import warn_no_remap
import message_filters
import numpy as np
import rospy
import sensor_msgs.msg

from jsk_perception.cfg import DrawRectsConfig


class DrawRects(ConnectionBasedTransport):

    def __init__(self):
        super(DrawRects, self).__init__()

        self.colors = np.array(np.clip(
            labelcolormap() * 255, 0, 255), dtype=np.uint8)
        self.subs = []
        self.use_classification_result = DrawRectsConfig.defaults[
            'use_classification_result']
        self.approximate_sync = DrawRectsConfig.defaults['approximate_sync']
        self.queue_size = DrawRectsConfig.defaults['queue_size']
        self._srv_dynparam = dynamic_reconfigure.server.Server(
            DrawRectsConfig, self._config_callback)

        self.pub_viz = self.advertise(
            '~output', sensor_msgs.msg.Image, queue_size=1)

    def _config_callback(self, config, level):
        need_resubscribe = False
        if self.use_classification_result != config.use_classification_result \
           or self.approximate_sync != config.approximate_sync \
           or self.queue_size != config.queue_size:
            need_resubscribe = True

        self.approximate_sync = config.approximate_sync
        self.queue_size = config.queue_size
        self.use_classification_result = config.use_classification_result
        self.show_proba = config.show_proba
        self.rect_boldness = config.rect_boldness

        self.font_path = config.font_path
        if self.use_classification_result and not osp.exists(self.font_path):
            rospy.logwarn('Not valid font_path: {}'.format(self.font_path))
        self.label_size = int(np.ceil(config.label_size))
        self.label_boldness = config.label_boldness
        self.label_font = config.label_font
        self.label_margin_factor = config.label_margin_factor

        self.resolution_factor = config.resolution_factor
        self.interpolation_method = config.interpolation_method

        if need_resubscribe and self.is_subscribed():
            self.unsubscribe()
            self.subscribe()
        return config

    def subscribe(self):
        sub_image = message_filters.Subscriber('~input', sensor_msgs.msg.Image)
        sub_rects = message_filters.Subscriber('~input/rects', RectArray)
        warn_no_remap('~input', '~input/rects')

        subs = [sub_image, sub_rects]
        if self.use_classification_result:
            sub_class = message_filters.Subscriber(
                '~input/class', ClassificationResult)
            subs.append(sub_class)

        if self.approximate_sync:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                subs,
                queue_size=self.queue_size, slop=slop)
            sync.registerCallback(self.draw_rects_callback)
        else:
            sync = message_filters.TimeSynchronizer(
                subs, queue_size=self.queue_size)
            sync.registerCallback(self.draw_rects_callback)
        self.subs = subs

    def unsubscribe(self):
        for sub in self.subs:
            sub.sub.unregister()

    def draw_rects_callback(self, img_msg, rects_msg, class_msg=None):
        bridge = cv_bridge.CvBridge()
        cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        img = cv2.resize(cv_img, None,
                         fx=self.resolution_factor,
                         fy=self.resolution_factor,
                         interpolation=self.interpolation_method)
        for i, rect in enumerate(rects_msg.rects):
            if self.use_classification_result:
                label_idx = class_msg.labels[i]
            else:
                label_idx = i

            color = self.colors[label_idx % len(self.colors)]

            pt1 = (int(rect.x * self.resolution_factor),
                   int(rect.y * self.resolution_factor))
            pt2 = (int((rect.x + rect.width) * self.resolution_factor),
                   int((rect.y + rect.height) * self.resolution_factor))
            cv2.rectangle(img, pt1=pt1, pt2=pt2,
                          color=color.tolist(),
                          thickness=self.rect_boldness,
                          lineType=cv2.LINE_AA)
            if self.use_classification_result and osp.exists(self.font_path):
                text = class_msg.label_names[i]
                if self.show_proba and len(class_msg.label_proba) > i:
                    text += ' ({0:.2f})'.format(class_msg.label_proba[i])
                pos_x = int(rect.x * self.resolution_factor)
                pos_y = int(rect.y * self.resolution_factor)
                pos = (pos_x, pos_y)
                img = put_text_to_image(
                    img, text, pos, self.font_path,
                    self.label_size,
                    color=(255, 255, 255),
                    background_color=tuple(color),
                    offset_x=self.rect_boldness / 2.0)
        viz_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        viz_msg.header = img_msg.header
        self.pub_viz.publish(viz_msg)


if __name__ == '__main__':
    rospy.init_node('draw_rects')
    dr = DrawRects()
    rospy.spin()
