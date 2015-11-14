# VoxelGridLargeScale

VoxelGrid downsampler which can handle small leaf_size.
Only supports `pcl::PointXYZ`.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`)

  Input cloud

## Publishing Topics
* `~output` (`sensor_msgs/PointCloud2`)

  Output downsampled cloud

## Parameters
* `~leaf_size` (default: `0.01`)

  Size of voxel grid.
