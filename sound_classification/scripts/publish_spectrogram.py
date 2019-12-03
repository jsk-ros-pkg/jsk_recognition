#!/usr/bin/env python

# This node publish spectrogram msg from spectrum msg

from cv_bridge import CvBridge
from sound_classification.msg import Spectrum, Volume, Wave
import matplotlib.cm as cm
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image


class PublishSpectrogram:

    def __init__(self):
        # init rospy node
        rospy.init_node('publish_spectrogram', anonymous=True)
        # config for spectrogram
        self.length = rospy.get_param('/microphone/length', 512)  # length relates hamming window range
        self.rate = rospy.get_param('/microphone/rate', 44100)
        self.cutoff_rate = rospy.get_param('/publish_spectrogram/cutoff_rate', self.rate/2)
        self.f = np.fft.fftfreq(self.length, d=1.0/self.rate)
        cutoff_f = self.f[self.f < self.cutoff_rate]
        cutoff_f = cutoff_f[cutoff_f > 0]
        rospy.loginfo('frequency list of fft:\n{}'.format(cutoff_f))
        self.hit_volume_thre = rospy.get_param('/publish_spectrogram/hit_volume_threshold', 0)
        self.visualize_data_length = min(
            int(self.length * self.cutoff_rate / self.rate), self.length/2)
        self.time_to_listen = rospy.get_param('/publish_spectrogram/time_to_listen', 0.3)
        self.queue_size = int(self.time_to_listen * (self.rate / self.length))
        self.wave_spec_queue = np.zeros((
            self.queue_size,
            self.visualize_data_length
            ))  # remove folding noise
        self.bridge = CvBridge()
        self.count_from_last_hitting = 0

        # publisher
        self.spectrogram_pub = rospy.Publisher(  # spectrogram (always published)
            '/microphone/spectrogram', Image)
        self.hit_spectrogram_pub = rospy.Publisher(  # spectrogram published only when big sound is detected
            '/microphone/hit_spectrogram', Image)

        # subscriber
        wave_sub = message_filters.Subscriber('/microphone/wave', Wave)
        spectrum_sub = message_filters.Subscriber('/microphone/sound_spec', Spectrum)
        vol_sub = message_filters.Subscriber('/microphone/volume', Volume)
        self.ts = message_filters.TimeSynchronizer([wave_sub, spectrum_sub, vol_sub], 10)
        self.ts.registerCallback(self.cb)

        # published msg
        self.img_msg = Image()

    def cb(self, wave, spectrum, vol):
        # calc spectrogram
        spec_data = np.array(spectrum.spectrum[:self.visualize_data_length])  # remove folding noise
        self.wave_spec_queue = np.concatenate([self.wave_spec_queue, spec_data[None]])
        self.wave_spec_queue = self.wave_spec_queue[1:]  # add new element to the queue
        normalized_spec_data = self.wave_spec_queue / np.max(self.wave_spec_queue)
        jet_img = np.array(cm.jet(1 - normalized_spec_data)[:, :, :3] * 255, np.uint8)
        jet_img_transposed = jet_img.transpose(1, 0, 2)[::-1]
        img_msg = self.bridge.cv2_to_imgmsg(jet_img_transposed, 'bgr8')
        # set stamp
        stamp = wave.header.stamp
        self.img_msg.header.stamp = stamp
        # publish msg
        self.spectrogram_pub.publish(img_msg)
        if vol.volume > self.hit_volume_thre:
            self.count_from_last_hitting = 0
        else:  # publish (save) hit_spectrogram a little after hitting
            if self.count_from_last_hitting == self.queue_size / 3:
                self.hit_spectrogram_pub.publish(img_msg)
        self.count_from_last_hitting += 1


if __name__ == '__main__':
    ps = PublishSpectrogram()
    rospy.spin()
