import rospy

from audio_to_spectrogram.data_buffer import DataBuffer


class AudioBuffer(DataBuffer):

    def __init__(self, topic_name='~audio',
                 input_sample_rate=16000,
                 window_size=10.0,
                 bitdepth=16,
                 n_channel=1, target_channel=0,
                 get_latest_data=False,
                 discard_data=False,
                 auto_start=False):

        super(AudioBuffer, self).__init__(
            topic_name=topic_name,
            expr_to_get_data='m.data',  # field of audio_common_msgs/AudioData
            input_sample_rate=input_sample_rate,
            window_size=window_size,
            is_integer=True,
            is_signed=True,
            bitdepth=bitdepth,
            n_channel=n_channel,
            target_channel=target_channel,
            get_latest_data=get_latest_data,
            discard_data=discard_data,
            auto_start=auto_start,
        )

    @staticmethod
    def from_rosparam(**kwargs):
        n_channel = rospy.get_param('~n_channel', 1)
        target_channel = rospy.get_param('~target_channel', 0)
        mic_sampling_rate = rospy.get_param('~mic_sampling_rate', 16000)
        bitdepth = rospy.get_param('~bitdepth', 16)
        return AudioBuffer(input_sample_rate=mic_sampling_rate,
                           bitdepth=bitdepth,
                           n_channel=n_channel,
                           target_channel=target_channel,
                           **kwargs)
