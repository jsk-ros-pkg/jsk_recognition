#!/usr/bin/env python

import os.path as osp

import cv2
import cv_bridge
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
        self.use_classification_result = rospy.get_param('~use_classification_result', DrawRectsConfig.defaults[
            'use_classification_result'])
        self.approximate_sync = rospy.get_param('~approximate_sync', DrawRectsConfig.defaults['approximate_sync'])
        self.queue_size = rospy.get_param('~queue_size', DrawRectsConfig.defaults['queue_size'])
        self.transport_hint = rospy.get_param('~image_transport', 'raw')
        rospy.loginfo("Using transport {}".format(self.transport_hint))
        #
        # To process latest message, we need to set buff_size must be large enough.
        # we need to set buff_size larger than message size to use latest message for callback
        # 640*480(image size) / 5 (expected compressed rate) *
        #            70 (number of message need to be drop 70 x 30msec = 2100msec processing time)
        #
        # c.f. https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        #
        self.buff_size = rospy.get_param('~buff_size', 640 * 480 * 3 // 5 * 70)
        rospy.loginfo("rospy.Subscriber buffer size : {}".format(self.buff_size))

        self.bridge = cv_bridge.CvBridge()

        self._srv_dynparam = dynamic_reconfigure.server.Server(
            DrawRectsConfig, self._config_callback)

        if self.transport_hint == 'compressed':
            self.pub_viz = self.advertise(
                '{}/compressed'.format(rospy.resolve_name('~output')), sensor_msgs.msg.CompressedImage, queue_size=1)
        else:
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
        if self.transport_hint == 'compressed':
            sub_image = message_filters.Subscriber('{}/compressed'.format(rospy.resolve_name('~input')), sensor_msgs.msg.CompressedImage, buff_size=self.buff_size)
        else:
            sub_image = message_filters.Subscriber('~input', sensor_msgs.msg.Image, buff_size=self.buff_size)
        sub_rects = message_filters.Subscriber('~input/rects', RectArray)
        warn_no_remap('~input', '~input/rects')

        subs = [sub_image, sub_rects]
        if self.use_classification_result:
            sub_class = message_filters.Subscriber(
                '~input/class', ClassificationResult)
            subs.append(sub_class)

        if self.approximate_sync:
            slop = rospy.get_param('~slop', 1.0)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                subs,
                queue_size=self.queue_size, slop=slop)
            self.sync.registerCallback(self.draw_rects_callback)
        else:
            self.sync = message_filters.TimeSynchronizer(
                subs, queue_size=self.queue_size)
            self.sync.registerCallback(self.draw_rects_callback)
        self.subs = subs
        rospy.loginfo("  approximate_sync : {}".format(self.approximate_sync))
        rospy.loginfo("  queue_size : {}".format(self.queue_size))

    def unsubscribe(self):
        for sub in self.subs:
            sub.sub.unregister()

    def draw_rects_callback(self, img_msg, rects_msg, class_msg=None):
        start_time = rospy.Time.now()
        if self.transport_hint == 'compressed':
            # decode compressed image
            np_arr = np.fromstring(img_msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_msg.format.find("compressed rgb") > -1:
                cv_img = cv_img[:, :, ::-1]
        else:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

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
        if self.transport_hint == 'compressed':
            viz_msg = sensor_msgs.msg.CompressedImage()
            viz_msg.format = "jpeg"
            viz_msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        else:
            viz_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        viz_msg.header = img_msg.header
        self.pub_viz.publish(viz_msg)

        rospy.loginfo("processing time {} on message taken at {} sec ago".format(
            (rospy.Time.now() - start_time).to_sec(),
            (rospy.Time.now() - img_msg.header.stamp).to_sec()))


if __name__ == '__main__':
    rospy.init_node('draw_rects')
    dr = DrawRects()
    rospy.spin()
