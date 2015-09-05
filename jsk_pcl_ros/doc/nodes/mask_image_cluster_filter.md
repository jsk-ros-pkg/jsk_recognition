# MaskImageClusterFilter
![](images/mask_image_cluster_indices_concatenator.png)

Segment Clouds with mask_image and Clustering Methods(example. SuperVoxel).

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input point cloud.
* `~target` (`jsk_recognition_msgs/ClusterPointIndices`)

  Result of Some Clustering methods
* `~input/mask` (`sensor_msgs/Image`)

  Mask image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameters of the image.

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Indices of the points masked with `~input/mask` and '~target'.
