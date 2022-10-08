audio_tagging.py
================


What is this?
-------------

Audio tagging, which classify input sounds, node using `PANNs inference <https://github.com/qiuqiangkong/panns_inference>`_.


Subscribing Topic
-----------------

* ``~audio`` (``audio_common_msgs/AudioData``)

  Input depth image.


Publishing Topic
----------------

* ``~output/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Class information of detected sounds.


Parameters
----------

* ``~classifier_name`` (String, default: ``rospy.get_name()``)

  Name of this classifier.

* ``~n_channel`` (Int, Default: ``1``)

  Number of input audio channel.

* ``~target_channel`` (Int, Default: ``0``)

  Target input audio channel.

* ``~mic_sampling_rate`` (Int, Default: ``16000``)

  Sample rate of input audio.

* ``~bitdepth`` (Int, Default: ``16``)

  Bitdepth of input audio.

* ``~gpu`` (Int, default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~rate`` (Int, Default: ``1``)

  Publishing rate [Hz].

* ``~window_size`` (Float, Default: ``1.0``)

  Window size of sound input. This value means the number of seconds of audio input.


Example
-------

.. code-block:: bash

   roslaunch sound_classification sample_audio_tagging.launch gpu:=-1 gui:=true
