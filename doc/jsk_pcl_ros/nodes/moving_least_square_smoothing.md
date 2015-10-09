# MovingLeastSquareSmoothing
## What Is This

This nodelet will subscribe the sensor\_msgs::PointCloud2, and publish the smoothed  sensor\_msgs::PointCloud2.
Please refer about MovingLeastSquare in http://pointclouds.org/documentation/tutorials/resampling.php

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.
## Publishing Topics

   Publish Smoothed sensor\_msgs::PointCloud2.

* `~output/point` (`sensor\_msgs::PointCloud2`)

## Parameters
* `~gauss_param_set` (Boolean, default: `False`)
  Whether set gauss param or not
* `~search_radius` (Double, default: `0.03`)
  Nearest Neighbor Search param
* `~use_polynomial_fit` (Boolean, default: `False`)
  Whether use polynomial_fit or not
* `~polynomial_order` (Int, default: `2`)
  Set polynomial order
* `~calc_normal` (Boolean, default: `True`)
  calc_normal
