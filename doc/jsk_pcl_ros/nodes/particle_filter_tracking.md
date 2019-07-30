# ParticleFilterTracking
## What Is This
![](images/particle_filter_tracking.png)

This nodelet tracks the target pointcloud.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

* `~input_change` (`sensor_msgs/PointCloud2`)

  Input change pointcloud, which is only enabled when `~use_change_detection` is true
  and should be synchronized with `~input`.

* `~renew_model` (`sensor_msgs/PointCloud2`)

  Reference pointcloud to track.

* `~renew_model_with_marker` (`visualization_msgs/Marker`)

  Reference marker model to track. This will convert marker model to pointcloud.
  You need to pass the marker whose type is `TRIANGLE_LIST` and it should have the color.

  This topic is subscribed only when `~align_box` is false.

* `~renew_box` (`jsk_recognition_msgs/BoundingBox`)

  Bounding box information to align reference pointcloud model.
  Available only if `~align_box` parameter is true, and it should be synchronized with `~renew_model`.

## Publishing Topic
* `~track_result` (`sensor_msgs/PointCloud2`)

  Reference pointcloud which is transformed by tracking result.

* `~tracking_result_pose` (`geometry_msgs/PoseStamped`)

  Tracking result as pose of reference pointcloud.

* `~particle` (`sensor_msgs/PointCloud2`)

  Particles during tracking. Only x, y and z are available.

* `~output/latest_time` (`std_msgs/Float32`)

  latest computation time

* `~output/average_time` (`std_msgs/Float32`)

  average computation time

* `~output/rms_angle_error` (`std_msgs/Float32`)
* `~output/rms_distance_error` (`std_msgs/Float32`)

  Root mean squared error of angle/distance.

* `~output/velocity` (`geometry_msgs/TwistStamped`)

  Velocity of object.

* `~output/velocity_norm` (`std_msgs/Float32`)

  Norm of velocity of object.

* `~output/no_move` (`std_msgs/Bool`)
* `~output/no_move_raw` (`std_msgs/Bool`)

  These topics will be true if object looks stable.

* `~output/skipped` (`std_msgs/Bool`)

  This topic is advertised but not published for now.

* `~output/change_marker` (`visualization_msgs/MarkerArray`)

  This topic is advertised only when `~use_change_detection` is true,
  but not published for now.

* `~output/tracker_status` (`jsk_recognition_msgs/TrackerStatus`)

  Current tracking status.

  This topic is published only when `~use_change_detection` is true.


## Advertising Servicies
* `~renew_model` (`jsk_recognition_msgs/SetPointCloud2`)

  Service interface to set reference pointcloud.

## Parameters
* `~thread_nr` (Integer, default: `cpu num`)

  The number of thread used in tracking
* `~particle_num` (Integer, default: `~max_particle_num - 1`)

  The number of initial particles
* `~use_normal` (Boolean, default: `false`)

  Use normal information to track or not.
* `~use_hsv` (Boolean, default: `true`)

  If it's true, tracker uses color information in HSV color space to
  evaluate likelihood.
* `~track_target_name` (String, default: `track_result`)

  The name of the target, it is used as frame_id of tf.
* `~octree_resolution` (Double, default: `0.01`)

  Octree resolution to search.
* `~align_box` (Bool, default: `false`)

  If it's true, tracker subscribes `~renew_box` topic and align reference model against the box.
* `~BASE_FRAME_ID` (String, default: `NONE`)

  Coordinate system of the tracker. `NONE` means "same to frame_id of input poiintcloud".
* `~default_initial_mean` (Array of double, default: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`)

  Mean value of initial sampling.
* `~initial_noise_covariance` (Array of double, default: `[0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]`)

  Covariance value of initial sampling.

* `~max_particle_num` (Integer, default: `1000`)

  Maximum numebr of particles

* `~iteration_num` (Integer, defeault: `1`)

  The number of iteration per one frame.
* `~resample_likelihood_thr` (Double, default: `0.0`)

  Threshold of likelihood to resample (re-initialize) all the particles.
* `~delta` (Double, default: `0.99`)

  Delta value for KLD sampling.
* `~epsilon` (Double, default: `0.2`)

  epsilon parameter for KLD sampling.
* `~bin_size_x` (Double, default: `0.01`)
* `~bin_size_y` (Double, default: `0.01`)
* `~bin_size_z` (Double, default: `0.01`)
* `~bin_size_roll` (Double, default: `0.01`)
* `~bin_size_pitch` (Double, default: `0.01`)
* `~bin_size_yaw` (Double, default: `0.01`)

  Size of bin for KLD sampling. Larger value means smaller number of particles.
* `~default_step_covariance_x` (Double, default: `0.0001`)
* `~default_step_covariance_y` (Double, default: `0.0001`)
* `~default_step_covariance_z` (Double, default: `0.0001`)
* `~default_step_covariance_roll` (Double, default: `0.004`)
* `~default_step_covariance_pitch` (Double, default: `0.004`)
* `~default_step_covariance_yaw` (Double, default: `0.004`)

  Covariance value of noise in resampling phase.

* `~reversed` (Boolean, default: `false`)

  Reverse relationship between reference and input. If this parameter is true,
  tracker transforms input pointcloud instead of reference pointcloud.
  It is useful when input pointcloud is smaller than reference pointcloud.

  If this parameter is true, KLDSampling is disabled.
* `~not_use_reference_centroid` (Boolean, default: `false`)

  If this parameter is true, tracker des not use centroid of reference pointcloud as the origin of reference pointcloud.

* `~not_publish_tf` (Boolean, default: `false`)

  If this parameter is true, do not publish tf frame.

* `~enable_cache` (Boolean, default: `false`)

  Enable caching of nearest-neighbor search.

  At most one of `~enable_cache` and `~enable_organized` can be set to true.

* `~enable_organized` (Boolean, default: `false`)

  Enable using Organized nearest-neighbor search

  At most one of `~enable_cache` and `~enable_organized` can be set to true.

* `~cache_size_x` (Double, default: `0.01`)
* `~cache_size_y` (Double, default: `0.01`)
* `~cache_size_z` (Double, default: `0.01`)

  Resolution of cache voxel grid.

  These parameters are enabled only when `~enable_cache` is true.

* `~max_distance` (Double, default: `0.01`)

  Maximum distance between points to take into account when computing likelihood

* `~use_change_detection` (Bool, default: `false`)

  Use change detection to skip tracking when no change in pointcloud.

* `~static_velocity_thr` (Double, default: `0.1`)

  Velocity threshold to regard object is stable.

* `~change_cloud_near_thr` (Double, default: `0.2`)

  Distance threshold to trigger tracking when `~use_change_detection` is true.

## Sample

```bash
roslaunch jsk_pcl_ros sample_particle_filter_tracking.launch
```

... or you can visualize tracking status by using
[tracking_info.py](tracking_info.md) and [tracker_status_info.py](tracker_status_info.md)

```bash
roslaunch jsk_pcl_ros sample_particle_filter_tracking_change_detection.launch
```

... or you can try service API [renew_tracking.py](renew_tracking.md)
to renew reference pointcloud instead of topic API.

```bash
roslaunch jsk_pcl_ros sample_particle_filter_tracking_service_renew.launch
```
