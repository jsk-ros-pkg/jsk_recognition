# MaskImageToPointIndices
A nodelet to convert mask image (`sensor_msgs::Image`) to `pcl_msgs/PointIndices` for
organized pointcloud.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask image.

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Output indices converted from the mask image.
