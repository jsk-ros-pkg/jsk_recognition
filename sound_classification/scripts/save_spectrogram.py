#!/usr/bin/env python

# This node saves spectrogram when big sound is detected

from cv_bridge import CvBridge
import os
import os.path as osp
from PIL import Image as Image_
import rospkg
import rospy
from sensor_msgs.msg import Image


class SaveSpectrogram:

    def __init__(self):
        rospy.init_node('save_spectrogram', anonymous=True)

        # config for saving spectrogram
        rospack = rospkg.RosPack()
        self.target_class = rospy.get_param(
            '~target_class', 'unspecified_data')
        self.save_dir = osp.join(rospack.get_path(
            'sound_classification'), 'train_data')
        self.image_save_dir = osp.join(
            self.save_dir, 'original_spectrogram', self.target_class)
        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)
        self.bridge = CvBridge()
        self.is_first = True

        # subscribe
        self.hit_spectrogram_sub = rospy.Subscriber(
            '/microphone/hit_spectrogram', Image, self.hit_spectrogram_cb)

    def hit_spectrogram_cb(self, msg):
        if self.is_first:  # do not save first spectrogram (noisy)
            self.is_first = False
            return
        file_num = len(os.listdir(self.image_save_dir)) + 1  # start from 00001.npy
        file_name = osp.join(self.image_save_dir, '{0:05d}.png'.format(file_num))
        # get image from rostopic
        hit_spectrogram = self.bridge.imgmsg_to_cv2(msg)
        Image_.fromarray(hit_spectrogram[:, :, [2, 1, 0]]).save(file_name)  # bgr -> rgb
        rospy.loginfo('save image: ' + file_name)


if __name__ == '__main__':
    ss = SaveSpectrogram()
    rospy.spin()
