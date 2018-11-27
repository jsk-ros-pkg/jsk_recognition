mask_rcnn_instance_segmentation.py
==================================

What is this?
-------------

.. image:: ./images/mask_rcnn_instance_segmentation.jpg

Predict object instance masks and labels.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.


Publishing Topic
----------------

* ``~output/cluter_indices`` (``jsk_recognition_msgs/ClusterPointIndices``)

  Image indices of each instance.

* ``~output/labels`` (``jsk_recognition_msgs/LabelArray``)

  Class labels of each instance.

* ``~output/label_cls`` (``sensor_msgs/Image``)

  Label image color-coded by class.

* ``~output/label_ins`` (``sensor_msgs/Image``)

  Label image color-coded by instance.

* ``~output/viz`` (``sensor_msgs/Image``)

  Visualized image of recognition result.

* ``~output/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Class information of detected objects

* ``~output/rects`` (``jsk_recognition_msgs/RectArray``)

  Rectangles of detected objects

Parameters
----------

* ``~gpu`` (Int, default: ``0``)

  GPU id.

* ``~score_thresh`` (Float, default: ``0.7``)

  Score threshold of detections.

* ``~fg_class_names`` (List of String, required)

  Foreground class names that is used to identify number of class.
  It is also used for the ``name`` field of ``~output/labels``.

* ``~pretrained_model`` (String, required)

  Pretrained model path.

* ``~classifier_name`` (String, default: ``rospy.get_name()``)

  Name of this classifier

Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_mask_rcnn_instance_segmentation.launch gpu:=0
