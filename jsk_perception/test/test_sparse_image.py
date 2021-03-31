#!/usr/bin/env python

import os.path as p
import cv2
from cv_bridge import CvBridge, CvBridgeError

import unittest

from sensor_msgs.msg import Image

import rospy


class EdgeImagePublisher(object):
    def __init__(self, imgpath):
        self.pub = rospy.Publisher("/test_edge_image", Image)
        self.sub_topic_name = "/sparse/image_decoded"
        self.sub = rospy.Subscriber(self.sub_topic_name, Image, self._callback)
        self.bridge = CvBridge()
        self.edgeimg = cv2.imread(imgpath)
        self.edgemsg = self.bridge.cv2_to_imgmsg(self.edgeimg, encoding="8UC3")
        self.response = False

    def _publish(self):
        try:
            self.edgemsg.header.stamp = rospy.Time.now()
            self.pub.publish(self.edgemsg)
        except CvBridgeError as e:
            print(e)

    def _check_img_msg_diff(self, msg1, msg2):
        cnt = 0
        if msg1.encoding == "8UC3":
            channel1 = 3
        else:
            channel1 = 1
        channel2 = 1
        if msg1.width != msg2.width:
            return -1
        if msg1.height != msg2.height:
            return -1
        if msg1.header.stamp != msg2.header.stamp:
            return -1
        receivedimg = self.bridge.imgmsg_to_cv2(msg2)
        for x in range(self.edgeimg.shape[1]):
            for y in range(self.edgeimg.shape[0]):
                if not (self.edgeimg[y,x] == receivedimg[y,x]).any():
                    rospy.loginfo(self.edgeimg[y,x])
                    rospy.loginfo(receivedimg[y,x])
                    rospy.loginfo("")
                    cnt += 1
        rospy.logdebug(cnt)
        return cnt

    def _callback(self, msg):
        if self._check_img_msg_diff(self.edgemsg, msg) == 0:
            self.response = True

    def checkSparseImage(self):
        cnt = 0
        while cnt <= 100:
            try:
                self._publish() 
                rospy.wait_for_message(self.sub_topic_name, Image, timeout=1.0)
            except:
                pass
            rospy.loginfo("waiting for subscribe...")
            if self.response:
                return True
            cnt += 1
        return False

class TestSparseImage(unittest.TestCase):
    def test_encode_decode_data16(self):
        imgpath = p.join(p.dirname(__file__), "../sample/edge_image_test_240x180.bmp")
        ei = EdgeImagePublisher(imgpath)
        self.assertTrue(ei.checkSparseImage())

    def test_encode_decode_data32(self):
        imgpath = p.join(p.dirname(__file__), "../sample/edge_image_test_640x480.bmp")
        ei = EdgeImagePublisher(imgpath)
        self.assertTrue(ei.checkSparseImage())


if __name__ == "__main__":
    import rostest
    rospy.init_node("sparse_image_test_py")
    rostest.rosrun("jsk_perception", "test_sparse_image", TestSparseImage)

