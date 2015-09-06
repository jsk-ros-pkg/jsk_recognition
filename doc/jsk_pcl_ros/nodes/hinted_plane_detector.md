# HintedPlaneDetector
## What Is This
![](images/hinted_plane_detector.png)
Estimate plane parameter from small 'hint' pointcloud and grow it to detect larger plane.

Algorithm is:

1. Detect hint plane from small hint pointcloud using RANSAC
2. Filter `~input` pointcloud based on distance and normal direction with hint plane.
3. Detect plane from the pointcloud using RANSAC
4. Segment clusters out of the inliers of the detected plane based on euclidean metrics
5. Apply density filter
6. Extract points from the nearest segmented clusters to the centroid of hint plane
7. Compute convex hull of the extracted points

## Subscribing Topic

* `~input` (`sensor_msgs/PointCloud`)

  Input pointcloud. It is required to have normal and xyz fields.

* `~input/hint/cloud` (`sensor_msgs/PointCloud`)

  Hint pointcloud to estimate plane parameter and only xyz fieleds are required.

## Publishing Topic

* `~output/polygon` (`geometry_msgs/PolygonStamped`)
* `~output/polygon_array` (`jsk_recognition_msgs/PolygonArray`)
* `~output/inliers` (`pcl_msgs/PointIndices`)
* `~output/coefficients` (`pcl_msgs/ModelCoefficients`)

  Result of detection.

* `~output/hint/polygon` (`geometry_msgs/PolygonStamped`)
* `~output/hint/polygon_array` (`jsk_recognition_msgs/PolygonArray`)
* `~output/hint/inliers` (`pcl_msgs/PointIndices`)
* `~output/hint/coefficients` (`pcl_msgs/ModelCoefficients`)

  Result of detection of hint pointcloud.
* `~output/polygon_before_filtering` (`geometry_msgs/PolygonStamped`)
* `~output/polygon_array_before_filtering` (`jsk_recognition_msgs/PolygonArray`)

  Result of detection before euclidean filtering.
* `~output/hint_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/plane_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/density_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/euclidean_filtered_indices` (`pcl_msgs/PointIndices`)

  Candidate point indices filtered by each filtering phase.

## Parameters
* `~hint_outlier_threshold`

  Outlier threshold to detect hint plane using RANSAC
* `~hint_max_iteration`

  Maximum iteration number to detect hint plane using RANSAC
* `~hint_min_size`

  Minimum number of inliers in hint plane
* `~outlier_threashold`

  Outlier threshold to detect larger plane using RANSAC
* `~max_iteration`

  Maximum iteration number to detect larger plane using RANSAC
* `~min_size`

  Minimum number of inliers in larger plane
* `~eps_angle`

  EPS angle to detect larger plane and normal direction is computed from hint plane.
* `~normal_filter_eps_angle`

  EPS angle to filter candidate points before detecting larger plane.
  Normal direction is computed from hint plane.
* `~euclidean_clustering_filter_tolerance`

  Tolerance distance in euclidean clustering to filter far points.
* `~euclidean_clustering_filter_min_size`

  Minimum cluster size in euclidean clustering to filter far points.

* `~density_radius` (Double, default: `0.1`)
* `~density_num` (Integer, default: `10`)

  These parameters are used in density filtering. The only points which have `~density_num` neighbors within
  `~density_radius` distance are passed.
