#!/usr/bin/env python3

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="torchvision.transforms.functional_tensor")

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg

import cv2
import numpy as np
import torch
from basicsr.archs.rrdbnet_arch import RRDBNet
from realesrgan import RealESRGANer

import threading
import queue
import os

class ImageSuperRes:
    def __init__(self):
        rospy.init_node('image_restoration_node', anonymous=True)

        input_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        output_topic = rospy.get_param('~output_topic', '/camera/restoration_image')

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(input_topic, Image, self.callback)
        self.pub = rospy.Publisher(output_topic, Image, queue_size=1)

        self.queue = queue.Queue(maxsize=10)
        self.lock = threading.Lock()

        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path('image_restoration'), 'weights', 'RealESRGAN_x4plus.pth')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.upsampler = RealESRGANer(
            scale=4,
            model_path=model_path,
            model=RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64,
                          num_block=23, num_grow_ch=32, scale=4),
            tile=0,
            tile_pad=10,
            pre_pad=0,
            half=True if torch.cuda.is_available() else False
        )

        self.upsampler.model.to(self.device)

        self.processing_thread = threading.Thread(target=self.process_images, daemon=True)
        self.processing_thread.start()

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.queue.full():
                self.queue.get()
            self.queue.put(cv_image)
        except Exception as e:
            rospy.logerr(f"Callback error: {e}")

    def process_images(self):
        while not rospy.is_shutdown():
            if not self.queue.empty():
                image = self.queue.get()
                try:
                    output, _ = self.upsampler.enhance(image)
                    output_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
                    self.pub.publish(output_msg)
                except Exception as e:
                    rospy.logerr(f"Super-resolution processing error: {e}")

if __name__ == '__main__':
    try:
        ImageSuperRes()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
