# split_image

![](./images/split_image.png)

## What Is This

Split the input image at equal intervals vertically and horizontally.

`split_image.py` is a python node.

If you want to use this with nodelet, use `split_image`.


## Subscribing Topic

* `~input` (`sensor_msgs/Image`)

    Input image to be split.

## Publishing Topic

* `~output/vertical(vertical_index)/horizontal(horizontal_index)` (`sensor_msgs/Image`)

    Split images. The number of published topics is `~vertical_splits` * `~horizontal_splits`.

## Parameters

* `~vertical_splits` (int, default: 1)

    The number of parts the input image is split vertically.

* `~horizontal_splits` (int, default: 1)

    The number of parts the input image is split horizontally.

## Parameters for nodelet version.

* `~queue_size` (int: default: `100`)

    Input queue size.

* `~vertical%02d/horizontal%02d/camera_info_url` (string: default: `""`)

    Path to camera info yaml.

    If you want to calibrate a camera, please see [camera_calibration](http://wiki.ros.org/camera_calibration).

* `~vertical%02d/horizontal%02d/frame_id` (string: default: `""`)

    Each frame id of split image message.

    If nothing is specified, `~input`'s `frame_id` is used.


## Sample
```bash
roslaunch jsk_perception sample_split_image.launch
```
