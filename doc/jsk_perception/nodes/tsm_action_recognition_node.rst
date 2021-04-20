tsm_action_recognition_node.py
==============================

What is this?
-------------

.. image:: ./images/tsm_action_recognition.gif


Inference actions for video.
Please refer to `original paper <https://arxiv.org/abs/1811.08383>`_.


Subscribing Topic
-----------------


* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Action class labels with scores.


Publishing Topic
----------------

* ``~output/viz`` (``sensor_msgs/Image``)

  Visualized image of recognition result.

* ``~output/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Class information of detected actions.


Parameters
----------

* ``~gpu`` (Int, default: ``-1``)

  GPU id.

* ``~class_names`` (List of String, required)

  Action class names. This value is used to identify number of class.

* ``~pretrained_model`` (String, required)

  Pretrained model path.

* ``~queue_size`` (Int, default: ``1``)

  How many messages you allow about the subscriber to keep in the queue.

* ``~classifier_name`` (String, default: ``rospy.get_name()``)

  Name of this classifier


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_tsm_action_recognition.launch gpu:=0
