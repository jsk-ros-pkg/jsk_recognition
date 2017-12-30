PointCloudToMaskImage
=====================

![pointcloud_to_mask_image](https://user-images.githubusercontent.com/1901008/34367264-aaa28e1e-eaeb-11e7-8699-c5e17dec71e8.gif)

What is this?
-------------

Convert point cloud to mask image with following rules:

1. NaN region of PointXYZ will be black (0).
2. Other region will be white (255).


Subscribing Topic
-----------------

* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

  * `~input/depth` (`sensor_msgs/Image`)

  Input depth image


Publishing Topic
----------------

* `~output` (`sensor_msgs/Image`)

  Output mask image.

Parameters
----------

* `~z_near` (`Double`, default: `0.0`)

  Lower limit of depth to be projected to mask image for each pixel (meter)

* `~z_far~ (`Double`, default: `10.0`)

  Upper limit of depth to be projected to mask image for each pixel (meter)
