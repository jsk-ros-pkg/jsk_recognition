# RemoveBlurredFrames
## What Is This
![](../images/remove_blurred_frames_demo.gif)

The node for removing blurred images from image topics. When a blurred image is subscribed, nothing is published, and when a non-blurred image is subscribed, it is published as is. It detects blurred images by thresholding the value of variance of a Laplacian filtered image.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

Image input.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

Non-blurred image output.

* `~output/mask` (`sensor_msgs/Image`)

Laplacian filtered image output.

* `~output/var` (`std_msgs/Float64`)

Laplacian filtered image's variance.

## Parameters
* `~min_laplacian_var` (Double, default: `400.0`)

The threshold of laplacian variance. Increasing this value removes more frames.

## For use

```bash
roslaunch jsk_perception remove_blurred_frames.launch INPUT_IMAGE:=<your image>
```

## Sample

```bash
roslaunch jsk_perception sample_remove_blurred_frames.launch
```

