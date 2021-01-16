# renew_tracking.py

![](images/particle_filter_tracking.png)

Call `jsk_recognition_msgs/SetPointCloud2` service from subscribed point cloud.

## Subscribing Topics

* `selected_pointcloud` (`sensor_msgs/PointCloud2`)

  Point cloud.


## Calling Services

* `particle_filter_tracker/renew_model` (`jsk_recognition_msgs/SetPointCloud2`)

  Set new point cloud as a callback of `selected_pointcloud`.


## Parameters

* `~track_target_name` (String, default: `track_result`)

  Dummy name used for calling service internally.


## Sample

```bash
roslaunch jsk_pcl_ros sample_particle_filter_tracking_service_renew.launch
```
