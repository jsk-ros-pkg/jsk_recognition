# add_camera_info.py

## What Is This

Add camera_info to an image without camera_info.

## Subscribing Topic

* `~input` (`sensor_msgs/Image`)

    Input image which does not have camera_info.

## Publishing Topic

* `~output` (`sensor_msgs/CameraInfo`)

    Output camera_info with the same time stamp as the input image

## Parameters

* `~yaml_filename` (string, default: '~/.ros/camera_info/camera.yaml')

    Path to the yaml file with camera_info information. The yaml file is supposed to be generated with the following command.
    ```bash
    $ rosrun camera_calibration cameracalibrator.py
    ```

## Sample
```bash
roslaunch jsk_perception sample_add_camera_info.launch
```
