evaluate_box_segmentation_by_gt_box.py
======================================

What is this?
-------------

.. image:: images/evaluate_box_segmentation_by_gt_box.gif

Evaluate three-dimensional box segmentation by a gt. (ground truth) bounding box.


Subscribing Topic
-----------------

* ``~input/box_gt`` (``jsk_recognition_msgs/BoundingBox``)

  GT. bounding box.

* ``~input/box`` (``jsk_recognition_msgs/BoundingBox``)

  Input bounding box.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/Accuracy``)

  Accuracy of box segmentation evaluated by gt. box.

.. math::

  volume^{gt} = Volume(box^{gt} \cap box)

  accuracy = \frac{volume^{gt}}{Volume(box^{gt}) + Volume(box) - volume^{gt}}


Sample
------

.. code-block:: bash

  roslaunch jsk_pcl_ros_utils sample_evaluate_box_segmentation_by_gt_box.launch
