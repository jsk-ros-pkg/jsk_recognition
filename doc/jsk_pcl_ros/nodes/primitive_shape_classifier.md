PrimitiveShapeClassifier
=========================

![primitive_shape_classifier](https://user-images.githubusercontent.com/1901008/27854434-ca088eee-61a1-11e7-9a98-0864013a8f63.png)

Classify shape for each cluster point indices on planes.

## Subscribing Topics

* `~input` (`sensor_msgs/PointCloud2`)

    Input XYZRGB point cloud

* `~input/normal` (`sensor_msgs/PointCloud2`)

    Input normal point cloud

* `~input/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Input cluster point indices

* `~input/polygons` (`jsk_recognition_msgs/PolygonArray`)

    Input supporting planes

## Publishing Topics

* `~output` (`jsk_recognition_msgs/ClassificationResult`)

    Classification result

* `debug/boundary_indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Cluster point indices of boundary points for each clustered objects

* `debug/projected_cloud` (`sensor_msgs/PointCloud2`)

    Projected boundary points on supporting planes

## Parameters

* `~queue_size` (`Int`, default: `100`)

    Queue size for message synchronization

* `~min_points_num` (`Int`, default: `10`)

    Minimum number of points for each cluster.

* `~sac_max_iterations` (`Int`, default: `500`)

    Maximum iteration number for SAC segmentation

* `~sac_distance_threshold` (`Double`, default: `0.005`)

    Distance threshold for SAC Segmentation

* `~sac_radius_limit_min` (`Double`, default: `0.025`)

    Minimum radius for circle estimation

* `~sac_radius_limit_max` (`Double`, default: `0.13`)

    Maximum radius for circle estimation

* `~box_threshold` (`Double`, default: `0.70`)

    Threshold to classify objects as boxes

* `~circle_threshold` (`Double`, default: `0.30`)

    Threshold to classify objects as circles
