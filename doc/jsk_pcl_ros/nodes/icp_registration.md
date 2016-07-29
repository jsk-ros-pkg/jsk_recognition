# ICPRegistration
![](../images/icp_registration.png)

Register two pointclouds based on icp like registration technique.

## Subscribe Topics
* `~input` (`sensor_msgs/PointCloud`)

  Target pointcloud.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera info.
* `~input_reference` (`sensor_msgs/PointCloud`)

  Reference pointcloud. frame_id of this pointlcoud is ignored.
* `~input_reference_array` (`jsk_recognition_msgs::PointsArray`)

  Array of reference pointcloud. ICPRegistration uses the reference
  which provides the best fitting score.
* `~input_offset` (`geometry_msgs/PoseStamped`)

  Offset of pose. It will only be subscribed if `~use_offset_pose` is true.
* `~input_reference_add` (`sensor_msgs/PointCloud`)
* `~input_box` (`jsk_recognition_msgs/BoundingBox`)
* `~reference` (`sensor_msgs/PointCloud`)

## Publishing Topics
* `~output` (`sensor_msgs/PointCloud`)
* `~output_pose` (`geometry_msgs/PoseStamped`)
* `~debug/source` (`sensor_msgs/PointCloud`)
* `~debug/target` (`sensor_msgs/PointCloud`)
* `~debug/flipped` (`sensor_msgs/PointCloud`)
* `~debug/result` (`sensor_msgs/PointCloud`)
* `~icp_result` (`jsk_recognition_msgs/ICPResult`)

* `~output/latest_time` (`std_msgs/Float32`)

  latest computation time

* `~output/average_time` (`std_msgs/Float32`)

  average computation time


## Advertising Services
* `~icp_align` (`jsk_recognition_msgs/ICPAlign`)
* `~icp_align_with_box` (`jsk_recognition_msgs/ICPAlignWithBox`)

## Parameters
* `~use_offset_pose` (default: `false`)

  Enable `~input_offset` topic.
* `~use_normal` (default: `false`)

  Use normal information in registration.
  In order to use this feature, reference and target pointcloud should have
  valid normal fields.
* `~align_box` (default: `false`)
* `~synchronize_reference` (default: `false`)

* `~algorithm` (default: `ICP`)

  Should be one of `ICP`, `GICP` or `NDT`.
* `~correspondence_algorithm` (default: `NN`)

  Should be one of `NN` or `Projective`.
* `~use_flipped_initial_pose` (default: `true`)
* `~max_iteration` (default: `100`)
* `~correspondence_distance` (default: `10`)
* `~transform_epsilon` (default: `1e-9`)
* `~euclidean_fittness_epsilon` (default: `0.01`)
* `~rotation_epsilon` (default: `2e-3`)
* `~ransac_iterations` (default: `1000`)
* `~ransac_outlier_threshold` (default: `0.05`)
* `~correspondence_randomness` (default: `20`)
* `~maximum_optimizer_iterations` (default: `20`)
* `~ndt_resolution` (default: `1.0`)
* `~ndt_step_size` (default: `0.05`)
* `~ndt_outlier_ratio` (default: `0.35`)
