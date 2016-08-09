# RGBDecomposer
![](images/rgb_decomposer_sample_image.png)

Split the original color image into 3 greyscale image of each color(RGB) strength.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Devide input camera image into 3 image (R,G,B) according to its color info.

## Publishing Topic
* `output/red` (`sensor_msgs/Image`)
* `output/green` (`sensor_msgs/Image`)
* `output/blue` (`sensor_msgs/Image`)

## Parameters
none

## Samples

### rosrun
Launch color image publisher anything you like.

```
rosrun usb_cam usb_cam_node
```
Run this rgb_decomposer node with proper remapping.

```
rosrun jsk_perception rgb_decomposer ~input:=/usb_cam/image_raw
```
