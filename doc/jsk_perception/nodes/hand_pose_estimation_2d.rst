hand_pose_estimation_2d.py
============================


What is this?
-------------

.. image:: ./images/srhandnet.png

Estimate hand pose in 2d.
Please refer to `original paper <https://www.yangangwang.com/papers/WANG-SRH-2019-07.html>`_.

In order to use this feature, you need to install `pytorch <https://pytorch.org/get-started/locally/>`_ (pytorch >= 1.4.0 is recommended).


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input image.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Detected hand pose image.

* ``~pose`` (``jsk_recognition_msgs/HandPoseArray``)

  Publish hand keypoints 2D positions and scores in image.


Parameters
----------

* ``~gpu`` (Int, Default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~thre1`` (Float, Default: ``0.3``)

  Threshold of hand bounding box heatmap value.

* ``~thre2`` (Float, Default: ``0.2``)

  Threshold of hand keypoint heatmap value.

* ``~thre3`` (Int, Default: ``5``)

  Threshold of undetected keypoints quantity.

* ``~visualize`` (Bool, Default: ``True``)

  If ``~visualize`` is true, draw estimated hand keypoints.

* ``~model_file`` (String, Required)

  Trained `SRHandNet model file <https://www.yangangwang.com/papers/WANG-SRH-2019-07.html>`_.


Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_hand_pose_estimation_2d.launch gpu:=0


Reference
---------

::

  @article{Wang:2019:SRH,
    doi = {10.1109/TIP.2019.2955280},
    title = {SRHandNet: Real-time 2D Hand Pose Estimation with Simultaneous Region Localization},
    journal = {IEEE Transactions on Image Processing},
    author = {Yangang Wang, Baowen Zhang and Cong Peng},
    number = 1,
    month = Oct.,
    volume = 29,
    year = 2019,
    pages = {2977 - 2986},
    url = {http://yangangwang.com/papers/WANG-SRH-2019-07.html},
  }
