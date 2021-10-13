#!/usr/bin/env python

import numpy as np
from os import makedirs
from os import path as osp

from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image


class NoiseSaver(object):
    """
    Collect noise spectrum (no_sound spectrum)
    which is used for both sound detection and noise subtraction

    Kill this node by Ctrl-c, then, noise.npy is saved
    """

    def __init__(self):
        # Config for loading noise data
        rospack = rospkg.RosPack()
        self.train_dir = osp.join(rospack.get_path(
            'sound_classification'), 'train_data')
        if not osp.exists(self.train_dir):
            makedirs(self.train_dir)
        self.noise_data_path = osp.join(self.train_dir, 'noise.npy')
        self.spectrums = np.array([])
        # ROS
        rospy.on_shutdown(self.save_noise_spectrum)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("~raw_spectrogram", Image, self.cb)
        self.spectrogram = None
        self.save_data_rate = rospy.get_param('~save_data_rate')
        rospy.Timer(rospy.Duration(1. / self.save_data_rate), self.timer_cb)

    def save_noise_spectrum(self):
        """
        This method is called when this node is killed by Ctrl-c
        save noise spectrum in 1 file, because it is easy to load
        """

        rospy.loginfo(
            'Saved noise.npy with {} data'.format(len(self.spectrums)))
        np.save(self.noise_data_path, self.spectrums)

    def cb(self, msg):
        self.spectrogram = msg

    def timer_cb(self, timer):
        """
        Main process of NoiseSaver class
        Append spectrum data to self.spectrums at self.save_data_rate
        """

        if self.spectrogram is None:
            return
        spectrogram = self.bridge.imgmsg_to_cv2(self.spectrogram)
        self.current_spectrum = np.mean(spectrogram, axis=1)
        if len(self.spectrums) == 0:
            self.spectrums = self.current_spectrum[None]
        else:
            self.spectrums = np.append(self.spectrums,
                                       self.current_spectrum[None],
                                       axis=0)
        rospy.loginfo('Save {} noise samples.'.format(len(self.spectrums)))


if __name__ == '__main__':
    rospy.init_node('noise_saver')
    n = NoiseSaver()
    rospy.spin()
