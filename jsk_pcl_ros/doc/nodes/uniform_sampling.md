# UniformSampling
Sample pointloud in the manner of uniform sampling.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Sampled indices

## Parameters
* `~search_radius` (Double, default: `0.01`)

  Sampling radius to apply uniform samplng.
