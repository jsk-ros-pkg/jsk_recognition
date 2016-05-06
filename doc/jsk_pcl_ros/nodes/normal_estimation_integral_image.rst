NormalEstimationIntegralImage
=============================


What is this?
-------------

.. image:: images/normal_estimation_integral_image.png

Compute normals for an organized point cloud using integral images.


Subscribing Topics
------------------

- ``~input`` (``sensor_msgs/PointCloud2``)

  Input point cloud. (``PointXYZRGB``)


Publishing Topics
-----------------

- ``~output`` (``sensor_msgs/PointCloud2``)

  Output normals.


Parameters
----------

- ``max_depth_change_factor`` (Double, default: 0.02)

  max depth change factor

- ``normal_smoothing_size`` (Double, default: 20.0)

  normal smoothing size parameter

- ``estimation_method`` (Int, default: 1)

  Estimation method.

  - 0: AVERAGE_3D_GRADIENT
  - 1: COVARIANCE_MATRIX
  - 2: AVERAGE_DEPTH_CHANGE

- ``depth_dependent_smoothing`` (Boolean, default: false)

  Use depth dependent smoothing.

- ``border_policy_ignore`` (Boolean, default: true)

  Ignore border policy.
