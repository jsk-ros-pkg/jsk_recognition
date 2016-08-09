# HSVDecomposer
![](images/hsv_decomposer_sample_image.png)

Split the original color image into 3 greyscale image of each color(HSV) strength.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Devide input camera image into 3 image (H,S,V) according to its color info.

## Publishing Topic
* `output/hue` (`sensor_msgs/Image`)
* `output/saturation` (`sensor_msgs/Image`)
* `output/value` (`sensor_msgs/Image`)

## Parameters
none

## Samples

### rosrun
Launch color image publisher anything you like.

```
rosrun usb_cam usb_cam_node
```
Run this hsv_decomposer node with proper remapping.

```
rosrun jsk_perception hsv_decomposer ~input:=/usb_cam/image_raw
```
