# MultiPlaneExtraction
![MultiPlaneExtraction](images/multi_plane_extraction.png)

Extract the points above the planes between `~min_height` and `~max_height`.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   Input pointcloud.
* `~indices` (`jsk_recognition_msgs/ClusterPointIndices`)
* `~input_polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~input_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

   The input planes. If `~use_indices` parameter is false, `~indices` will not be used.

## Publishing Topics
* `~output` (`sensor_msgs/PointCloud2`):

   Pointcloud above the planes between `~min_height` and `~max_height`.
* `~output_nonplane_cloud` (`sensor_msgs/PointCloud2`):

   Pointcloud above the planes is not between `~min_height` and `~max_height`.
* `~output/indices` (`pcl_msgs/PointIndices`)

  PointIndices of points which are between `~min_height` and `~max_height`.

## Parameters
* `~keep_organized` (Bool, default: `false`)

   Keep organized point cloud or not.

* `~min_height` (Double, default: `0.0`)
* `~max_height`(Double, default: `0.5`)

   Minimum and maximum height of 3-D polygonal region to extract points.
* `~max_queue_size` (Integer, default: `100`)

   Queue length for subscribing topics.
* `~use_indices` (Bool, default: `True`)

   Use indices of planar regions to filter if it's set true.
   You can disable this parameter to filter pointcloud which is not the same pointcloud
   to segment planes
* `~magnify` (Double, default: `0.0`)

   Magnify planes by this parameter. The unit is m.
* `~use_async` (Boolean, default: `False`)

  Approximate sync input topics.
* `~use_sensor_frame` (Boolean, default: `False`)

  Use sensor viewpoint
* `~sensor_frame` (String, default: `head_root`)

  Specify frame\_id of sensor origin.
