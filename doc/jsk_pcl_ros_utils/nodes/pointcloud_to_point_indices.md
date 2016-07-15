# PointCloudToPointIndices

A nodelet to convert PointCloud (`sensor_msgs::PointCloud2`) to `pcl_msgs/PointIndices`.

## Subscribing Topic

* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud.

## Publishing Topic

* `~output` (`pcl_msgs/PointIndices`)

  Output indices converted from the pointcloud.
