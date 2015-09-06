# OrganizedPointCloudToPointIndices
A nodelet to convert organized PointCloud (`sensor_msgs::PointCloud2`) to `pcl_msgs/PointIndices` for
organized pointcloud.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input organized pointcloud.

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Output indices converted from the organized pointcloud.
