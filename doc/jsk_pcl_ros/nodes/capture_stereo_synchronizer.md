# CaptureStereoSynchronizer
## What Is This
![](images/capture_stereo_synchronizer.png)

A nodelet to capture training data of stereo cameras. It subscribe several messages with
synchronizing timestamp and republish them into `~output` namespace.

## Subscribing Topic
* `~input/pose` (`geometry_msgs/PoseStamped`)

  Pose of checkerboard

* `~input/mask` (`sensor_msgs/Image`)

  Mask image of the object

* `~input/mask_indices` (`pcl_msgs/PointIndices`)

  Pointcloud indices of the object

* `~input/left_image` (`sensor_msgs/Image`)

  Left camera image

* `~input/left_camera_info` (`sensor_msgs/CameraInfo`)

  Left camera parameter

* `~input/right_camera_info` (`sensor_msgs/CameraInfo`)

  Right camera parameter

* `~input/disparity` (`stereo_msgs/Disparity`)

  Disparity image of the stereo camear.

## Publishing Topic
* `~output/pose` (`geometry_msgs/PoseStamped`)
* `~output/mask` (`sensor_msgs/Image`)
* `~output/mask_indices` (`pcl_msgs/PointIndices`)
* `~output/left_image` (`sensor_msgs/Image`)
* `~output/left_camera_info` (`sensor_msgs/CameraInfo`)
* `~output/right_camera_info` (`sensor_msgs/CameraInfo`)
* `~output/disparity` (`stereo_msgs/Disparity`)

  These topics are the same message to the `~input/foo` messages but all of them
  are republished only if input messages are synchronized.

## Sample
Please check [capture_multisense_training_data.launch](launch/capture_multisense_training_data.launch).
