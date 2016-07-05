PointIndicesToClusterPointIndices
=================================


What is this?
-------------

Convert point indices to cluster point indices.


Subscribing Topic
-----------------

* ``~input`` (``pcl_msgs/PointIndices``)

  Point indices.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClusterPointIndices``)

  Cluster point indices, number of whose elements is 1 and it is the input indices.
