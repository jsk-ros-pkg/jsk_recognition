people_pose_estimation_2d.py
============================


What is this?
-------------

.. image:: ./images/people_pose_estimation_2d.png

Estimate people pose in 2d.
Please refer to `https://arxiv.org/abs/1611.08050`


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input image.

* ``~input/depth`` (``sensor_msgs/Image``)

  Input depth image.

* ``~input/info`` (``sensor_msgs/CameraInfo``)

  Input camera info.

Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Detected people pose image.

* ``~pose`` (``jsk_recognition_msgs/PeoplePoseArray``)

  If ``with_depth`` is true, publish 3D joint position.

  If ``with_depth`` is false, publish 2D joint position in image.

* ``~pose2d`` (``jsk_recognition_msgs/PeoplePoseArray``)

  If ``with_depth`` is true, publish 2D joint position.

Parameters
----------

* ``~gpu`` (Int, Default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~scales`` (Float, Default: ``0.38``)

  Resize image scale.

* ``~stride`` (Int, Default: ``8``)

  Stride of image.

* ``~pad_value`` (Int, Default: ``128``)

  Value of padding area.

* ``~thre1`` (Float, Default: ``0.1``)

  Threshold of heatmap value.

* ``~thre2`` (Float, Default: ``0.05``)

  Threshold of score.

* ``~visualize`` (Bool, Default: ``True``)

  If ``~visualize`` is true, draw an estimated pose.

* ``~model_file`` (String, Required)

  Trained model file.

* ``~with_depth`` (Bool, Default: ``False``)

  If true, subscribe ``~input/depth`` and ``~input/info``.

* ``~sync_camera_info`` (Bool, Default: ``False``)

  Synchronize ``~input/info`` if enabled, otherwise the last received camera info message is used.

* ``~approximate_sync`` (Bool, Default: ``True``)

  Use approximate synchronization policy.

* ``~queue_size`` (Int, Default: ``10``)

  Queue size for synchronization.

* ``~slop`` (Float, Default: ``0.1``)

  Slop for approximate sync.


Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_people_pose_estimation_2d.launch GPU:=0
