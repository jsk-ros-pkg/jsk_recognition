# VoxelGridDownsampleManager

## What Is This
![](images/voxel_grid_downsample.png)

Filter input point cloud by input marker size, and then downsample it.


## Subscribing Topic

* `~input` (`sensor_msgs/PointCloud2`)

  Original point cloud.

* `~add_grid` (`visualization_msgs/Marker`)

  Additional marker used for xyz pass through filter before downsampling.

  Only `frame_id`, `pose/position` and `scale` field are used for filtering.

  Also, leaf size [m] in voxel grid downsampling will be defined by `color/r` field.

  Below marker is added internally by default.

  ```yaml
  header:
    frame_id: ~base_frame
  pose:
    position:
      x: 2.0
      y: 0.0
      z: -0.5
  scale:
    x: 4.0
    y: 2.0
    z: 3.0
  color:
    r: 0.05
  ```


## Publishing Topic

* `~output` (`sensor_msgs/PointCloud2`)

  Downsampled point cloud for debugging.

* `~output_encoded` (`jsk_recognition_msgs/SlicedPointCloud`)

  Downsampled and sliced point cloud.

  All sliced clusters will be published in order with the `slice_index` info.

  Number of the clusters is calculated from `~max_points`.

  Messages of this topic can be decoded by `jsk_pcl_ros/VoxelGridDownsampleDecoder`.


## Parameter

* `~base_frame` (String, default: `pelvis`)

  Frame ID of initial marker.

* `~max_points` (Int, default: `300`)

  Number of maximum points in `~output_encoded`.

* `~rate` (Float, default: `1.0`)

  Multiplicative inverse of duration between publishing `output_encoded`.

  The unit is [Hz].


## Sample

```bash
roslaunch jsk_pcl_ros sample_voxel_grid_downsample.launch
```
