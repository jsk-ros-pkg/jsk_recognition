#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv2
import cv_bridge
from geometry_msgs.msg import PolygonStamped
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
import jsk_recognition_utils
import message_filters
from skimage.color import rgb_colors


class DrawRectArray(ConnectionBasedTransport):
    def __init__(self):
        super(DrawRectArray, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self._sub = message_filters.Subscriber('~input', Image)
        self.use_async = rospy.get_param('~approximate_sync', False)
        self.queue_size = rospy.get_param('~queue_size', 100)
        self.slop = rospy.get_param('~slop', 0.1)
        self._subscribe_rect_array()
        self._subscribe_polygon()

    def _subscribe_rect_array(self):
        self._sub_rects = message_filters.Subscriber(
            '~input/rect_array', RectArray)
        subs = [self._sub, self._sub_rects]
        if self.use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, self.queue_size, self.slop)
        else:
            sync = message_filters.TimeSynchronizer(subs, self.queue_size)
        sync.registerCallback(self._draw)

    def _subscribe_polygon(self):
        self._sub_polygon = message_filters.Subscriber(
            '~input/polygon', PolygonStamped)
        subs = [self._sub, self._sub_polygon]
        if self.use_async:
            sync = message_filters.ApproximateTimeSynchronizer(
                subs, self.queue_size, self.slop)
        else:
            sync = message_filters.TimeSynchronizer(subs, self.queue_size)
        sync.registerCallback(self._draw_polygon)

    def unsubscribe(self):
        self._sub.unregister()
        self._sub_rects.unregister()

    def _draw_polygon(self, imgmsg, polygon_msg):
        rect_msg = Rect()
        x1 = polygon_msg.polygon.points[0].x
        y1 = polygon_msg.polygon.points[0].y
        x2 = polygon_msg.polygon.points[1].x
        y2 = polygon_msg.polygon.points[1].y
        rect_msg.x = int(x1)
        rect_msg.y = int(y1)
        rect_msg.width = int(x2 - x1)
        rect_msg.height = int(y2 - y1)
        rects = [rect_msg]
        rects_msg = RectArray(header=polygon_msg.header, rects=rects)
        self._draw(imgmsg, rects_msg)

    def _draw(self, imgmsg, rects_msg):
        n_colors = min(len(rects_msg.rects), 256)
        cmap = jsk_recognition_utils.color.labelcolormap(N=n_colors)

        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')

        for i, r in enumerate(rects_msg.rects):
            color = cmap[i % n_colors]
            color = (color * 255).astype(np.integer)
            color = (color[2], color[1], color[0])  # RGB -> BGR
            pt1 = (r.x, r.y)
            pt2 = (r.x + r.width, r.y + r.height)
            cv2.rectangle(img, pt1, pt2, color)

        out_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = imgmsg.header
        self._pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('draw_rect_array')
    app = DrawRectArray()
    rospy.spin()
