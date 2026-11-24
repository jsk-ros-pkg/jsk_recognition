#!/usr/bin/env python

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import DepthImageFilterConfig as Config
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image


class DepthImageFilter(ConnectionBasedTransport):

    def __init__(self):
        super(DepthImageFilter, self).__init__()

        self.bridge = cv_bridge.CvBridge()
        self.srv = Server(Config, self.config_callback)
        self.pub_mask = self.advertise(
            '~output/mask', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self.callback,
                                    queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def config_callback(self, config, level):
        self.depth_threshold = config.threshold
        self.negative = config.negative
        return config

    def callback(self, depth_img_msg):
        bridge = self.bridge

        supported_encodings = {'16UC1', '32FC1'}
        if depth_img_msg.encoding not in supported_encodings:
            rospy.logwarn('Unsupported depth image encoding: {0}'
                          .format(depth_img_msg.encoding))

        depth = bridge.imgmsg_to_cv2(depth_img_msg)
        if depth_img_msg.encoding == '16UC1':
            depth = depth / 1000.0  # convert metric: mm -> m

        mask = np.zeros(depth.shape, dtype=np.uint8)
        mask_idx = np.logical_and(
            0 < depth,
            depth < self.depth_threshold)
        if self.negative is True:
            mask_idx = np.logical_not(mask_idx)
        mask[mask_idx] = 255

        mask = (mask).astype(np.uint8)
        mask_msg = bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = depth_img_msg.header
        self.pub_mask.publish(mask_msg)


if __name__ == '__main__':
    rospy.init_node('depth_image_filter')
    node = DepthImageFilter()
    rospy.spin()
