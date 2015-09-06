# GeometricConsistencyGrouping
Estimate model position using Geometric Consisteny Grouping technique

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Scene pointcloud. The type is `pcl::PointNormal`.
* `~input/feature` (`sensor_msgs/PointCloud2`)

  Scene feature. currently SHOT352 is supported.

* `~input/reference` (`sensor_msgs/PointCloud2`)

  Model pointcloud. The type is `pcl::PointNormal`.
* `~input/reference/feature` (`sensor_msgs/PointCloud2`)

  Model feature. currently SHOT352 is supported.

## Publishing Topic
* `~output` (`geometry_msgs/PoseStamped`)

  Pose of recognized object

## Parameters
* `~gc_size` (Double, default: `0.01`)

  Size of cluster
* `~gc_thresh` (Double, default: `5.0`)

  Threshold of clustering
