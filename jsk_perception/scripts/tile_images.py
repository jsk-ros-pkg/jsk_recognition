#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
from StringIO import StringIO

import cv2
import PIL
import numpy as np
from matplotlib import pyplot as plt

import rospy
import cv_bridge
import message_filters
from sensor_msgs.msg import Image


def tile_construction(img_num):
    x_num = 0
    y_num = int(math.sqrt(img_num))
    while x_num * y_num < img_num:
        x_num += 1
    return x_num, y_num


def tile_images(imgs):
    img_rgb_list = []
    for img in imgs:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_rgb_list.append(img_rgb)

    x_num, y_num = tile_construction(len(imgs))
    for i, img_rgb in enumerate(img_rgb_list):
        plt.subplot(y_num, x_num, i+1)
        plt.axis('off')
        plt.imshow(img_rgb)

    f = StringIO()
    plt.savefig(f, facecolor=(.9, .9, .9))
    out_rgb = np.array(PIL.Image.open(f))
    out_bgr = cv2.cvtColor(out_rgb, cv2.COLOR_RGB2BGR)
    plt.close()
    return out_bgr


def callback(*msgs):
    bridge = cv_bridge.CvBridge()
    imgs = []
    for msg in msgs:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        imgs.append(img)
    out_bgr = tile_images(imgs)
    imgmsg = bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
    pub_img.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node('tile_images')

    input_topics = rospy.get_param('~input_topics', [])
    if not input_topics:
        rospy.logerr('need to specify input_topics')
        sys.exit(1)

    pub_img = rospy.Publisher('~output', Image, queue_size=1)

    sub_img_list = []
    for i, input_topic in enumerate(input_topics):
        sub_img = message_filters.Subscriber(input_topic, Image)
        sub_img_list.append(sub_img)
    async = message_filters.ApproximateTimeSynchronizer(sub_img_list, 10, 10)
    async.registerCallback(callback)

    rospy.spin()
