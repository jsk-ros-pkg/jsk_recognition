# image_publisher.py

![](./images/image_publisher.png)

Publish image from loaded file.

## Publishing Topics
* `~output` (`sensor_msgs/Image`)

* `~output/camera_info` (`sensor_msgs/CameraInfo`)

## Parameters
* `~file_name` (str default: `image.png`)

  full path to the file to be loaded
* `publish_info` (bool default: `True`)

  publish `~output/camera_info` if true

* `~encoding` (str default: `bgr8`)
* `~frame_id` (str default: `camera`)

## Sample

```bash
roslaunch jsk_perception sample_image_publisher.launch
```
