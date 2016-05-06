# DepthCalibration
## What Is This

This nodelet applies calibration model to depth image.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input depth image.
* `~camera_info` (`sensor_msgs/CameraInfo`)

  Input camera_info.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Output depth image.

## Parameter
* `~coefficients2` (Array of double, default: `[0, 0, 0, 0, 0]`)

coefficients of calibration model.
* `~coefficients1` (Array of double, default: `[0, 0, 0, 0, 1.0]`)

coefficients of calibration model.
* `~coefficients0` (Array of double, default: `[0, 0, 0, 0, 0]`)

coefficients of calibration model.
* `~use_abs` (Boolean, default: `False`)

If you want to use absolute value in applying calibration model, please set this variable True.
* `~uv_scale` (Double, default: `1.0`)

If you want to scale value in applying calibration model, please set this variable.

## Sample
```
roslaunch jsk_pcl_ros depth_calibration.launch
```
