evaluate_voxel_segmentation_by_gt_box.py
========================================

What is this?
-------------

.. image:: images/evaluate_voxel_segmentation_by_gt_box.gif

Evaluate three-dimensional voxel segmentation by a gt. (ground truth) bounding box.


Subscribing Topic
-----------------

* ``~input/box_gt`` (``jsk_recognition_msgs/BoundingBox``)

  GT. bounding box.

* ``~input/markers`` (``visualization_msgs/MarkerArray``)

  Input voxel.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/Accuracy``)

  Accuracy of voxel segmentation evaluated by gt. box.

.. math::

  accuracy = \frac{Volume(voxels_{tp})}{Volume(box^{gt}) + Volume(voxels) - Volume(voxels_{tp})}


Sample
------

.. code-block:: bash

  roslaunch jsk_pcl_ros_utils sample_evaluate_voxel_segmentation_by_gt_box.launch
