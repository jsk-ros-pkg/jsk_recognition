LabelToClusterPointIndices
==========================

What is this?
-------------

.. image:: ./images/label_to_cluster_point_indices.png

Convert label image to cluster point indices with assumption of 0 is background label and
it is published as an another topic.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``, encoding: `32SC1`)

  Label image.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClusterPointIndices``)

  Set of point indices and each point indices means each label.
  label value ``0`` is recognized as background label, so ``i(index of cluster point indices) + 1`` = ``label number``.
  (ex. point indices in index ``0`` is region where label value is ``1``)
