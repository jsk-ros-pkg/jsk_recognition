# image_time_diff.py

![](../images/image_time_diff_hue_s.jpg)
![](../images/image_time_diff_saturation_s.jpg)
![](../images/image_time_diff_diff_s.jpg)

Publish difference between current input image and stored one. 

Image is stored when start msgs is subscribed.

## Subscribing Topics
* `~input/hue` (`sensor_msgs/Image`)
* `~input/saturation` (`sensor_msgs/Image`)

  Input images.

* `~start` (`std_msgs/Header`)

  Store input image and start comparing.

* `~stop` (`std_msgs/Header`)

  Release stored image and stop comparing.

## Publishing Topics
* `~output/diff` (`jsk_recognition_msgs/ImageDifferenceValue`)
* `~output/diff_image` (`sensor_msgs/Image`)

## Parameters
* `rate` (Float, default: `10`)

  Comparing rate [Hz].

* `~saturation_threshold` (int, default: `0`)

  Threshold of saturation to get diff of hue.

  This parameter can be changed by `dynamic_reconfigure`.


Sample
------

```
roslaunch jsk_perception sample_image_time_diff.launch
```
