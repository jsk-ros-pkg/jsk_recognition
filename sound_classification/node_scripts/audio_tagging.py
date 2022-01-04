#!/usr/bin/env python

import os

import jsk_recognition_msgs.msg
import numpy as np
from panns_inference import AudioTagging
from panns_inference import labels
import rospy
import torch

from sound_classification import AudioStream


class AudioTaggingNode(object):

    def __init__(self):
        super(AudioTaggingNode, self).__init__()

        self.classifier_name = rospy.get_param(
            "~classifier_name", rospy.get_name())
        self.n_channel = rospy.get_param('~n_channel', 1)
        self.target_channel = rospy.get_param('~target_channel', 0)
        self.sampling_rate = rospy.get_param('~mic_sampling_rate', 16000)
        bitdepth = rospy.get_param('~bitdepth', 16)
        if bitdepth == 16:
            self.dtype = 'int16'
        else:
            rospy.logerr("'~bitdepth' {} is unsupported.".format(bitdepth))
        self.window_size = rospy.get_param('~window_size', 1.0)
        self.audio_buffer_len = int(self.sampling_rate * self.window_size)

        # setup audio tagging.
        gpu_id = rospy.get_param('~gpu', -1)
        if gpu_id < 0:
            device = 'cpu'
        else:
            os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu_id)
            torch.cuda.set_device('cuda:{}'.format(gpu_id))
            device = 'cuda'
        self.at = AudioTagging(checkpoint_path=None, device=device)

        # Publisher and Subscriber
        self.stream = AudioStream(
            '~audio',
            input_sample_rate=self.sampling_rate,
            output_sample_rate=32000,
            depth=bitdepth,
            n_channel=self.n_channel,
            target_channel=self.target_channel,
            get_latest_data=True)

        self.pub_results = rospy.Publisher(
            '~output/class',
            jsk_recognition_msgs.msg.ClassificationResult,
            queue_size=1)

        # set time callback.
        rate = rospy.get_param('~rate', 1)
        if rate == 0:
            rospy.logwarn('You cannot set 0 as the rate; change it to 1.')
            rate = 1
        rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)

    def timer_cb(self, timer):
        if self.stream.sufficient_data(self.audio_buffer_len):
            return

        audio = self.stream.read(size=self.audio_buffer_len,
                                 normalize=True)
        audio = audio[None, :]
        scores, _ = self.at.inference(audio)
        scores = scores[0]

        # normalize scores.
        probs = scores / np.sum(scores)
        # sort labels
        indices = np.argsort(probs)

        # publish classification result message.
        result_msg = jsk_recognition_msgs.msg.ClassificationResult()
        result_msg.header.stamp = rospy.Time.now()
        result_msg.labels = indices
        result_msg.label_names = np.array(labels)[indices]
        result_msg.target_names = labels
        result_msg.label_proba = probs[indices]
        result_msg.probabilities = probs
        result_msg.classifier = self.classifier_name
        self.pub_results.publish(result_msg)


if __name__ == '__main__':
    rospy.init_node('audio_tagging_node')
    audio_tagging = AudioTaggingNode()  # NOQA
    rospy.spin()
