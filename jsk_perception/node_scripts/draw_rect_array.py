#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import cv2
import cv_bridge
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray
import message_filters
from skimage.color import rgb_colors


class DrawRectArray(ConnectionBasedTransport):
    def __init__(self):
        super(DrawRectArray, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

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
        sync.registerCallback(self._draw)

    def unsubscribe(self):
        self._sub.unregister()
        self._sub_rects.unregister()

    def _draw(self, imgmsg, rects_msg):
        colors = [v for v in rgb_colors.__dict__.values()
                  if isinstance(v, tuple)]

        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')

        for i, r in enumerate(rects_msg.rects):
            color = np.array(colors[i % len(colors)][:3])  # RGBA -> RGB
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
