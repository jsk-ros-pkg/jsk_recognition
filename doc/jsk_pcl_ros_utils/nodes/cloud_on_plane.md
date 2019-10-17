CloudOnPlane
===========

![](images/cloud_on_plane.png)

Publishes true when a pointcloud is on a polygon.

## Subscribing Topics

* `~input` (`sensor_msgs/PointCloud2`)
* `~input/polygon` (`jsk_recognition_msgs/PolygonArray`)

  Input pointcloud and polygons.


## Publishing Topics

* `~output` (`jsk_recognition_msgs/BoolStamped`)

  True if distance between pointcloud and polygon is smaller than
  `~distance_thr` for `~buf_size` frames.


## Parameters
* `~approximate_sync` (Bool, default: `False`)

  Whether to allow approximate synchronization for input topics.

* `~distance_thr` (Float, default: `0.05`)

  Distance threshold between pointcloud and polygon.

  This parameter can be changed by `dynamic_reconfigure`.

* `~buf_size` (Int, default: `2`)

  CloudOnPlane only returns true if all the recent `~buf_size` results is true.

  This parameter can be changed by `dynamic_reconfigure`.


## Sample

```bash
roslaunch jsk_pcl_ros_utils sample_cloud_on_plane.launch
```
