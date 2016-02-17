# TorusFInder
![](images/torus_finder.png)

Find a torus out of pointcloud based on RANSAC with 3-D circle model.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud`)

  Input pointcloud. You may need to choose good candidates of pointcloud.
* `~input/polygon` (`geometry_msgs/PolygonStampedd`)

  Input polygon. You can use `geometry_msgs/PolygonStampedd` instead of `sensor_msgs/PointCloud`

## Publishing Topic

* `~output` (`jsk_recognition_msgs/Torus`)

  Output of detection.

* `~output/inliers` (`pcl_msgs/PointIndices`)
* `~output/coefficients` (`pcl_msgs/ModelCoefficients`)

  Inliers and coefficients which represents detection result.
* `~output/array` (`jsk_recognition_msgs/TorusArray`)

  Array of torus. It will be used for visualization.

* `~output/posie` (`geometry_msgs/PoseStamped`)

  Publish result of detection as `geometry_msgs/PoseStamped`

* `~output/with_failure` (`jsk_recognition_msgs/Torus`)
* `~output/with_failure/array` (`jsk_recognition_msgs/Torus`)

  Output of detection with failure information.

* `~output/latest_time` (`std_msgs/Float32`)

  latest computation time

* `~output/average_time` (`std_msgs/Float32`)

  average computation time


## Parameters
* `~min_radius` (Double, default: `0.1`)
* `~max_radius` (Double, default: `1.0`)

  Minimum and maximum radius of torus.
* `~min_size` (Integer, default: `10`)

  Minimum number of inliers.
* `~outlier_threshold` (Double, default: `0.01`)

  Outlier threshold used in RANSAC.
* `~max_iterations` (Integer, default: `100`)

  Maximum number of iterations of RANSAC.
