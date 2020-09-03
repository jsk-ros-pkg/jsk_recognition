face_pose_estimation.py
============================


What is this?
-------------

.. image:: https://user-images.githubusercontent.com/1901008/31308440-f7ffb6a6-abb1-11e7-8b89-0a332001448b.png

Estimate people face pose / gender from RGB Image and PeoplePoseArray.
Please refer to Hyperface_.

.. _Hyperface: https://arxiv.org/abs/1603.01249


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input image.

* ``~input/pose_2d`` (``jsk_recognition_msgs/PeoplePoseArray``)

  Input people pose in 2D input image.

* ``~input/pose`` (``jsk_recognition_msgs/PeoplePoseArray``)

  Input people pose in 3D.

Publishing Topic
----------------

* ``~output/pose`` (``geometry_msgs/PoseArray``)

  Estimated face poses for each people.

* ``~output/gender`` (``jsk_recognition_msgs/ClassificationResult``)

  Estimated gender with confidence for each people.

* ``~output/rects`` (``jsk_recognition_msgs/RectArray``)

  Face rectangles in 2D input image used for cropping.

These 3 topics are need to be synchronized.

Parameters
----------

* ``~gpu`` (Int, Default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~classifier_name`` (String, Default: ``rospy.get_name()``)

  Name of this classifier

* ``~approximate_sync`` (Bool, Default: ``False``)

  Use approximate synchronization policy.

* ``~queue_size`` (Int, Default: ``100``)

  Queue size for synchronization.

* ``~slop`` (Double, Default: ``0.1``)

  Slop for approximate sync.

* ``~face_padding`` (Double, Default: ``0.0``)

  Padding size factor for face rectangles.

* ``~face_threshold`` (Double, Default: ``0.5``)

  Threshold for confidence of detected faces.

Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_face_pose_estimation.launch gpu:=0
