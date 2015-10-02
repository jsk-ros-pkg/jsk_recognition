# ClusterPointIndicesDecomposer
![](images/bounding_box.png)
## What is this
Decompose `jsk_recognition_msgs/ClusterPointIndices` into array of topics of `sensor_msgs/PointCloud` like `~output00`, `~output01` and so on.
It also publishes tf of centroids of each cluster and oriented bounding box of them. The direction of the bounding box are aligned on to the nearest planes if available.

## Subscribing topics
* `~input` (`sensor_msgs/PointCloud2`):

   Input pointcloud.
* `~target` (`jsk_recognition_msgs/ClusterPointIndices`):

   Input set of indices to represent clusters.
* `~align_planes` (`jsk_recognition_msgs/PolygonArray`):
* `~align_planes_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

   The planes for bounding box to be aligned on.

## Publishing topics
* `~output%02d` (`sensor_msgs/PointCloud2`):

   Series of topics for each pointcloud cluster.
* `~debug_output` (`sensor_msgs/PointCloud2`):

   Concatenate all the clusters into one pointcloud and colorize each cluster to see the result of segmentation.
* `~boxes` (`jsk_recognition_msgs/BoundingBoxArray`):

   Array of oriented bounding box for each segmented cluster.

## Parameters
* `~publish_tf` (Boolean, default: `True`):

   Toggle tf publishing.
* `~publish_clouds` (Boolean, default: `True`):

   Toggle `~output%02d` topics.
* `~align_boxes` (Boolean, default: `False`):

   If this parameter is set to `True`, `~align_planes` and
   `~align_planes_coefficients` are enabled.
* `~use_pca` (Boolean, default: `False`):

   Run PCA algorithm on each cluster to estimate x and y direction.
* `~force_to_flip_z_axis` (Boolean, default: `False`)

   Flip z axis direction if this value is true.
