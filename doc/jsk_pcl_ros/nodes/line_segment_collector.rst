LineSegmentCollector
======================


.. image:: images/line_segment_collector.png


What is this?
-------------

Collect line segments.


Subscribing Topics
------------------

- ``~input`` (``sensor_msgs/PointCloud2``)

  Input pointcloud.

- ``~input_indices`` (``jsk_recognition_msgs/ClusterPointIndices``)

- ``~input_coefficients`` (``jsk_recognition_msgs/ModelCoefficientsArray``)

- ``~trigger`` (``jsk_recognition_msgs/TimeRange``)

Publishing Topics
-----------------

- ``~output/cloud`` (``sensor_msgs/PointCloud2``)

  Pointcloud of collectored line segments.

- ``~output/coefficients`` (``jsk_recognition_msgs/ModelCoefficientsArray``)

- ``~output/inliers`` (``jsk_recognition_msgs/ClusterPointIndices``)

- ``~output/polygons`` (``jsk_recognition_msgs/PolygonArray``)

- ``~debug/connect_segments/inliers`` (``jsk_recognition_msgs/ClusterPointIndices``)

Parameters
----------

- ``~segment_connect_normal_threshold`` (``Double``, default: ``0.9``)

- ``~ewma_tau`` (``Double``, default: ``0.2``)

- ``~outlier_threshold`` (``Double``, default: ``0.01``)

- ``~max_iterations`` (``Int``, default: ``100``)

- ``~min_indices`` (``Int``, default:``1000``)
