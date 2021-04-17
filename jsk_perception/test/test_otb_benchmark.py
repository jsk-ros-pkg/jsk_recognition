#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2021, JSK Robotics Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import cv2
from cv_bridge import CvBridge
from glob import glob
from jsk_recognition_msgs.msg import RectArray, Rect
import os
import re
import rospy
import rostest
from sensor_msgs.msg import Image
import sys
import time
import unittest

class IOUTest():

    def __init__(self):
        self.img_dir = rospy.get_param('~img_dir', None)
        self.gt_file = rospy.get_param('~gt_file', None)
        self.iou_thresh = rospy.get_param('~iou_thresh', 0.5)
        self.bridge = CvBridge()

        self.img_names = sorted(glob(os.path.join(self.img_dir, '*')))
        # print(self.img_names)

        with open(self.gt_file, 'r') as f:
            self.gt_rects = [list(map(float, re.split('[,\t ]', x.strip()))) for x in f.readlines()]

        # print(self.gt_rects)
        self.image_pub = rospy.Publisher(
            '~test_img', Image, queue_size=1)
        self.init_rect_pub = rospy.Publisher(
            '~init_rect', RectArray, queue_size=1)

        self.result_rect_sub = rospy.Subscriber('~output_rect', RectArray, self.rectCb)
        self.pred_rects = []
        self.pred_rects.append(self.gt_rects[0])

        time.sleep(1)

        stamp = rospy.Time.now()
        img = cv2.imread(self.img_names[0])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)

        rect_msg = RectArray()
        rect_msg.header.stamp = stamp
        rect = self.gt_rects[0]
        rect_msg.rects.append(Rect(rect[0], rect[1], rect[2], rect[3]))
        self.init_rect_pub.publish(rect_msg)

        time.sleep(1)
        img = cv2.imread(self.img_names[1])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)

    def BenchmarkIoUCheck(self):

        while len(self.pred_rects) < len(self.img_names) and not rospy.is_shutdown():
            # print("wait for result, {}".format(len(self.pred_rects)))
            rospy.sleep(1.0)

        if len(self.pred_rects) == len(self.gt_rects):
            self.mean_iou = self.compute_mean_iou()
            print("mean iou of {} is {}".format(self.img_names[0].rsplit("/",1)[0], self.mean_iou))

            if self.mean_iou < self.iou_thresh:
                rospy.logerr("the mean IoUT {} is lower than threshold {}".format(self.mean_iou, self.iou_thresh))
                return False
            else:
                return True

        else:
            return True

    def rectCb(self, msg):
        # print(msg.rects[0])
        rect = msg.rects[0]
        self.pred_rects.append([rect.x, rect.y, rect.width, rect.height])

        if len(self.pred_rects) >= len(self.img_names):
            return
        stamp = rospy.Time.now()
        img = cv2.imread(self.img_names[len(self.pred_rects)])
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = stamp
        self.image_pub.publish(msg)

    def compute_mean_iou(self):
        mean_iou = []
        for pred, gt in zip(self.pred_rects, self.gt_rects):
            area_a = pred[2] * pred[3]
            area_b = gt[2] * gt[3]
            iou_x1 = max(pred[0], gt[0])
            iou_y1 = max(pred[1], gt[1])
            iou_x2 = min(pred[0] + pred[2], gt[0] + gt[2])
            iou_y2 = min(pred[1] + pred[3], gt[1] + gt[3])
            iou_w = iou_x2 - iou_x1
            iou_h = iou_y2 - iou_y1
            if iou_w < 0 or iou_h < 0:
                mean_iou.append(0.0)
            else:
                area_iou = iou_w * iou_h
                iou = area_iou / (area_a + area_b - area_iou)
                mean_iou.append(iou)
        return sum(mean_iou)/len(mean_iou)


class BenchmarkTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("benchmark_test")

    def test_iou(self):
        # step1: check the init form convergence
        iou_test = IOUTest()
        self.assertTrue(iou_test.BenchmarkIoUCheck(), msg = 'Cannot reach the desired IoU results for benchmark tracking sample')


if __name__ == '__main__':
    try:
        rostest.run('rostest', 'trtr_benchmark_test', BenchmarkTest, sys.argv)
    except KeyboardInterrupt:
        pass

    print("exiting")

