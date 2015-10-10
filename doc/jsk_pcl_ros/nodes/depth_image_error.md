# DepthImageError

Compute error of depth image and corner point of checker board.

## Sample

```
$ roslaunch jsk_pcl_ros depth_error.launch
```

Use with multisense and murooka board.
```
$ roslaunch jsk_pcl_ros depth_error.launch IMAGE_TOPIC:=/multisense_local/left/image_rect_color CAMERA_INFO_TOPIC:=/multisense_local/left/camera_info GRID_SIZE_X:=6 GRID_SIZE_Y:=5 DEPTH_IMAGE:=/multisense/depth
```
