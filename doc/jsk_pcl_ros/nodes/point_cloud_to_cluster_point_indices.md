# PointCloudToClusterPointIndices
Just convert pointcloud to `jsk_recognition_msgs/ClusterPointIndices`.
This nodelet is useful to compute bounding box of pointcloud by ClusterPointIndicesDecomposer.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

## Publishing Topic
* `~output` (`jsk_recognition_msgs/ClusterPointIndices`)

  Output cluster indices.
