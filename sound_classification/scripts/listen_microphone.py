#!/usr/bin/env python

# This node publishes wave, spectrogram and volume topics from microphone

from sound_classification.msg import Spectrum, Volume, Wave
import numpy as np
import os.path as osp
import pyaudio
import rospkg
import rospy
import sys


class ListenMicrophone:

    def __init__(self):
        # init rospy node
        rospy.init_node('listen_microphone', anonymous=True)
        self.p = pyaudio.PyAudio()
        # config for microphone
        self.microphone_name = rospy.get_param('/microphone/name', 'default')
        self.length = rospy.get_param('/microphone/length', 512)  # length relates hamming window range
        self.rate = rospy.get_param('/microphone/rate', 44100)
        self.channels = 1
        self.format = pyaudio.paFloat32
        rospack = rospkg.RosPack()
        file_path = osp.join(rospack.get_path(
            'sound_classification'), 'scripts', 'mean_noise_sound.npy')
        if osp.exists(file_path):
            self.mean_noise_sound = np.load(file_path)
        else:
            rospy.logerr('create mean noise sound by rosrun sound_classification save_noise_sound.py')
            exit()
        # search for microphone
        self.device_index = True
        for index in range(0, self.p.get_device_count()):
            device_info = self.p.get_device_info_by_index(index)
            if self.microphone_name in device_info['name']:
                self.device_index = device_info['index']
        if self.device_index is True:
            print('Cannot find audio device!')
            sys.exit()
        # config for fft
        self.data = np.zeros((self.length, self.channels))
        self.window = np.hamming(self.length)
        self.stream = self.p.open(format=self.format,
                                  channels=self.channels,
                                  rate=self.rate,
                                  input=True,
                                  output=False,
                                  input_device_index=self.device_index,
                                  frames_per_buffer=self.length)

        # publisher
        self.wave_pub = rospy.Publisher(  # sound wave data, the length is self.length
            '/microphone/wave', Wave, queue_size=1)
        self.spectrum_raw_pub = rospy.Publisher(  # sound spectrum, which is fft of wave data
            '/microphone/sound_spec_raw', Spectrum, queue_size=1)
        self.spectrum_pub = rospy.Publisher(  # sound spectrum, which is fft of wave data
            '/microphone/sound_spec', Spectrum, queue_size=1)
        self.vol_pub = rospy.Publisher(  # current volume
            '/microphone/volume', Volume, queue_size=1)

        # published msg
        self.wave_msg = Wave()
        self.spec_raw_msg = Spectrum()
        self.spec_msg = Spectrum()
        self.vol_msg = Volume()

    def process(self):
        stamp = rospy.Time.now()
        tmp = self.stream.read(self.length)  # sound input -> float32 array
        data = np.fromstring(tmp, np.float32)
        self.data = np.array(data)

        # calc wave
        wave = self.data
        self.wave_msg.wave = wave
        self.wave_msg.header.stamp = stamp

        # calc volume
        vol = np.sqrt(np.mean(self.data**2))  # effective value
        self.vol_msg.volume = vol
        self.vol_msg.header.stamp = stamp

        # calc spectrum
        spec = np.abs(np.fft.fft(wave*self.window))
        self.spec_raw_msg.spectrum = spec
        self.spec_raw_msg.header.stamp = stamp
        try:
            spec = spec - self.mean_noise_sound
            spec = np.where(spec > 0, spec, self.mean_noise_sound * 0.01)  # Spectral Subtraction method
        except ValueError:
            rospy.logwarn('mean_noise_sound.npy may be old. $ roslaunch sound_classification save_noise_sound.launch')
        self.spec_msg.spectrum = spec
        self.spec_msg.header.stamp = stamp

        # publish msg
        self.wave_pub.publish(self.wave_msg)
        self.vol_pub.publish(self.vol_msg)
        self.spectrum_raw_pub.publish(self.spec_raw_msg)
        self.spectrum_pub.publish(self.spec_msg)

    def destruct(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.process()
        except rospy.ROSInterruptException:
            self.destruct()


if __name__ == '__main__':
    lm = ListenMicrophone()
    lm.run()
