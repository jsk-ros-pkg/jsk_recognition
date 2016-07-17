PointCloudToMaskImage
=====================


What is this?
-------------

Convert point cloud to mask image with following rules:

1. NaN region of PointXYZ will be black (0).
2. Other region will be white (255).


Subscribing Topic
-----------------

* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud


Publishing Topic
----------------

* `~output` (`sensor_msgs/Image`)

  Output mask image.
