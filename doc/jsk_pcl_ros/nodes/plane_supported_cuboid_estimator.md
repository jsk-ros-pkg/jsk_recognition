# PlaneSupportedCuboidEstimator
![](images/plane_supported_cuboid_estimator.png)

Estimate a cuboid on a plane. Plane information is used as hint.
It uses particle filter to estimate pose of cuboid.

`jsk_pcl/InteractiveCuboidLikelihood` is a helper nodelet
to confirm likelihood function behaves as expected.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud
* `~fast_input` (`sensor_msgs/PointCloud2`)

  Fater input pointcloud
* `~input/polygon` (`jsk_recognition_msgs/PolygonArray`)
* `~input/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

  Planes which may support cuboid object

## Publishing Topics
* `~output/result` (`jsk_recognition_msgs/BoundingBoxArray`)

  Result of estimation as bonding box.
* `~output/particles` (`sensor_msgs/PointCloud2`)

  Particles as pointcloud (xyzi)
* `~output/histogram/global/x` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/global/y` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/global/z` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/global/roll` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/global/pitch` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/global/yaw` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/dx` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/dy` (`jsk_recognition_msgs/HistogramWithRange`)
* `~output/histogram/dz` (`jsk_recognition_msgs/HistogramWithRange`)

  Histograms of particles for each dimension

## Advertising Services
* `~reset` (`std_srvs/Empty`)

  Reset particles filters.

## Parameters
* `~init_local_position_z_min`
* `~init_local_position_z_max`
* `~use_init_world_position_z_model`
* `~init_local_orientation_roll_mean`
* `~init_local_orientation_roll_variance`
* `~init_local_orientation_pitch_mean`
* `~init_local_orientation_pitch_variance`
* `~init_local_orientation_yaw_mean`
* `~init_local_orientation_yaw_variance`
* `~init_global_orientation_yaw_mean`
* `~init_global_orientation_yaw_variance`
* `~init_dx_mean`
* `~init_dx_variance`
* `~init_dy_mean`
* `~init_dy_variance`
* `~init_dz_mean`
* `~init_dz_variance`
* `~particle_num`
* `~step_x_variance`
* `~step_y_variance`
* `~step_z_variance`
* `~step_roll_variance`
* `~step_pitch_variance`
* `~step_yaw_variance`
* `~step_dx_variance`
* `~step_dy_variance`
* `~step_dz_variance`
* `~use_range_likelihood` (default: `False`)

  Set to true if you want to update likelihood based on geometry respected to plane.
* `~range_likelihood_local_min_z`
* `~range_likelihood_local_max_z`

  Allowed inimum and maximum distance from plane.
* `~use_occlusion_likelihood` (default: `False`)

  Take occlusion into acount when compute likelihood.
* `~min_inliers` (default: `10`)

  Minimum number of inliers.
* `~outlier_distance` (default: `0.01`)

  Threshold to regard points as inlier.
* `~sensor_frame` (default: `odom`)

  Frame ID of sensor frame. It is used to compute viewpoint and occlusion.
* `~fast_cloud_threshold` (default: `2.0`)

  If distance to object is smaller than
  `~fast_cloud_threshold`, `~fast_input` will be used.

* `~use_global_init_yaw` (default: `False`)

  Enable `~init_global_orientation_yaw_mean` and `~init_global_orientation_yaw_variance`
  to decide intial particle states.
  
