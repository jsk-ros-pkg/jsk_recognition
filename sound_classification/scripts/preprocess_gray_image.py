#!/usr/bin/env python

import cv_bridge
import numpy as np
import os.path as osp
import rospkg
import rospy
from sensor_msgs.msg import Image
from topic_tools import LazyTransport
from sound_classification.process_gray_image import spectral_subtract, normalize_gray_image


class PreprocessGrayImage(LazyTransport):
    """
    This class is to preprocess gray spectrogram for classifying.
    1. Spectral subtraction by spectral subtraction method
    # 2. Smooth spectrogram
    # 3. Normalize spectrogram (32FC1 -> 8UC1, make each pixel value 0 ~ 255)
    """

    def __init__(self):
        super(self.__class__, self).__init__()
        # Noise subtraction
        rospack = rospkg.RosPack()
        self.train_dir = osp.join(rospack.get_path(
            'sound_classification'), 'train_data')
        self.noise_data_path = osp.join(self.train_dir, 'noise.npy')
        if osp.exists(self.noise_data_path):
            noise_data = np.load(self.noise_data_path)
            self.mean_spectrum = np.mean(noise_data, axis=0)
        else:
            rospy.logwarn('{} is not found.'.format(self.noise_data_path))
            self.mean_spectrum = 0
        # Publisher and Subscriber
        self.bridge = cv_bridge.CvBridge()
        self.pub = self.advertise('~output', Image, queue_size=1)
        self.pub_normalized = self.advertise(
            '~output_normalized', Image, queue_size=1)
        self.subscribe()

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', Image, self._process,
                                    queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def _process(self, imgmsg):
        raw_img = self.bridge.imgmsg_to_cv2(imgmsg)
        # Spectral subtract
        subtracted_img = spectral_subtract(raw_img, self.mean_spectrum)
        # Normalize
        normalized_img = normalize_gray_image(subtracted_img)
        # 32FC1 -> 8UC1
        normalized_img = normalized_img.astype(np.uint8)
        # Publish
        # Spectral subtracted image
        pubmsg = self.bridge.cv2_to_imgmsg(subtracted_img)
        pubmsg.header = imgmsg.header
        self.pub.publish(pubmsg)
        # Normalized img
        pubmsg = self.bridge.cv2_to_imgmsg(normalized_img)
        pubmsg.header = imgmsg.header
        self.pub_normalized.publish(pubmsg)


if __name__ == '__main__':
    rospy.init_node('preprocess_gray_image')
    pgi = PreprocessGrayImage()
    rospy.spin()
