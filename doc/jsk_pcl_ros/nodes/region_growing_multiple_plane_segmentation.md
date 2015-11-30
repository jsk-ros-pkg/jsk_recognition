# RegionGrowingMultiplePlaneSegmentation
![jsk_pcl/RegionGrowingMultiplePlaneSegmentation](images/region_growing_multiple_plane_segmentation.png).

jsk\_pcl/RegionGrowingMultiplePlaneSegmentation estimates multiple plenes from pointcloud.


It extracts planes based on [region growing](http://en.wikipedia.org/wiki/Region_growing)
and evaluation function of connectivity if based on the following equation:
![](images/region_growing_multiple_plane_segmentation_eq.gif)

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.
* `~input_normal` (`sensor_msgs/PointCloud2`):

   normal pointcloud of `~input`

## Publishing Topics
* `~output/clustering_result` (`jsk_recognition_msgs/ClusterPointIndices`):

  Result of region growing as cluster.
* `~output/inliers` (`jsk_recognition_msgs/ClusterPointIndices`):

  Set of indices of the polygons.
* `~output/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

  Array of coefficients of the polygons.
* `~output/polygons` (`jsk_recognition_msgs/PolygonArray`):

  Polygons

* `~output/latest_time` (`std_msgs/Float32`)

  latest computation time

* `~output/average_time` (`std_msgs/Float32`)

  average computation time


## Parameters
* `~angular_threshold` (Double, default: `0.04`)

   Angular threshold to connect two points in one cluster. See
* `~distance_threshold` (Double, default: `0.1`)

   Distance threshold to connect two points in one cluster.
* `~max_curvature` (Double, default: `0.1`)

   Before extracting planes, filtering out the points which have higer curvature than this value.
* `~min_size` (Integer, default: `100`)

   The minimum number of the points of each plane.
* `~min_area` (Double, default: `0.1`)

   The minimum area of the convex areas.
* `~max_area` (Double, default: `100`)

   The max area of the convex areas.
* `~cluster_tolerance` (Double, default: `0.1`)

   The spatial tolerance for new cluster candidates.
* `~ransac_refine_outlier_distance_threshold` (Double, default: `0.1`)

   Outlier threshold for plane estimation using RANSAC.
* `~ransac_refine_max_iterations` (Integer, default: `100`)

   The maximum number of the iterations for plane estimation using RANSAC.
