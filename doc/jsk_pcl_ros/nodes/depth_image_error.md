# DepthImageError

Compute error of depth image and corner point of checker board.

## Subscribing Topic
* `~image` (`sensor_msgs/Image`)
* `~camera_info` (`sensor_msgs/CameraInfo`)
  Input image and camera info.
* `~point` (`geometry_msgs/PointStamped`)

  Corner point of checkerboard.

## Publishing Topic
* `~output` (`jsk_recognition_msgs/DepthErrorResult`)

  Error between depth image and `~point`.

## Parameters
* `~approximate_sync` (boolean, default: `false`)

  Use approximate sync if it is true.

## Sample

```
$ roslaunch jsk_pcl_ros depth_error.launch
```

Use with multisense and murooka board.
```
$ roslaunch jsk_pcl_ros depth_error.launch IMAGE_TOPIC:=/multisense_local/left/image_rect_color CAMERA_INFO_TOPIC:=/multisense_local/left/camera_info GRID_SIZE_X:=6 GRID_SIZE_Y:=5 DEPTH_IMAGE:=/multisense/depth
```
