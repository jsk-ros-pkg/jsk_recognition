# ParticleFilterTracking
## What Is This
![](images/particle_filter_tracking.png)

This nodelet tracks the target pointcloud.

##### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

* `~renew_model` (`sensor_msgs/PointCloud2`)

  Reference pointcloud to track.

* `~renew_model_with_marker` (`visualization_msgs/Marker`)

  Reference marker model to track. This will convert marker model to pointcloud.
  You need to pass the marker whose type is TRIANGLE_LIST and it should have the color.

* `~renew_box` (`jsk_recognition_msgs/BoundingBox`)

  Bounding box information to align reference pointcloud model. Only if availabel `~align_box` parameter is true.

##### Publishing Topic
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


##### Advertising Servicies
* `~renew_model` (`jsk_pcl_ros/SetPointCloud2`)

  Service interface to set reference pointcloud.

##### Parameters
* `~thread_nr` (Integer, default: `cpu num`)

  The number of thread used in tracking
* `~particle_num` (Integer, default: `~max_particle_num`)

  The number of initial particles
* `~use_normal` (Boolean, default: `false`)

  Use normal information to track or not.
* `~use_hsv` (Boolean, default: `true`)

  If it's true, tracker uses color information in HSV color space to
  evaluate likelihood.
* `~track_target_name` (Boolean, default: `track_result`)

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
* `~delta` (Double, default: `0.09`)

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
* `~default_step_covariance_x` (Double, default: 0.00001)
* `~default_step_covariance_y` (Double, default: 0.00001)
* `~default_step_covariance_z` (Double, default: 0.00001)
* `~default_step_covariance_roll` (Double, default: 0.00001)
* `~default_step_covariance_pitch` (Double, default: 0.00001)
* `~default_step_covariance_yaw` (Double, default: 0.00001)

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

  Enable caching of nearest-neighbor search

* `~enable_organized` (Boolean, default: `false`)

  Enable using Organized nearest-neighbor search

* `~cache_size_x` (Double, default: `0.01`)
* `~cache_size_y` (Double, default: `0.01`)
* `~cache_size_z` (Double, default: `0.01`)

  Resolution of cache voxel grid.

* `~max_distance` (Double, default: `1.0`)

  Maximum distance between points to take into account when computing likelihood

## Sample

run the below command.

```
roslaunch jsk_pcl_ros tracking_groovy.launch # (When use groovy)
roslaunch jsk_pcl_ros tracking_hydro.launch  #(When use hydro)
```

Push the "Select" button at the top bar , drag and surround the target poincloud which you want to track in the rectangle area.Then, finally, push the "SelectPointCloudPublishActoin" button at SelectPointCloudPublishAction Panel. The tracker will start tracking the target.
