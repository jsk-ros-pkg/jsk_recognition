# FeatureRegistration
Align pointcloud using 3d feature. Currently only FPFH is supported.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

   Input pointcloud. The type of point is `pcl::PointNormal`.
* `~input/feature` (`sensor_msgs/PointCloud2`)

   Input feature. The type of point is `pcl::FPFHSignature33`.
* `~input/reference/cloud` (`sensor_msgs/PointCloud2`)

   Reference pointcloud. The type of point is `pcl::PointNormal`.
* `~input/reference/feature` (`sensor_msgs/PointCloud2`)

   Reference feature. The type of point is `pcl::FPFHSignature33`.

## Publishing Topic
* `~output` (`geometry_msgs/PoseStamped`)

  Transformation to align reference cloud to input cloud.
* `~output/cloud` (`sensor_msgs/PointCloud`)

  Reference pointCloud which is aligned to input cloud.

## Parameters
* `~max_iterations` (Integer, default: `1000`)

  Maximum number of iterations.
* `~correspondence_randomness` (Integer, default: `2`)

  Number of nearest features to use
* `~similarity_threshold` (Double, default: `0.9`)

  Polygonal edge length similarity threshold
* `~max_correspondence_distance` (Double, default: `0.0075`)

  inlier threshold
* `~inlier_fraction` (Double, default: `0.25`)

  inlier fraction
