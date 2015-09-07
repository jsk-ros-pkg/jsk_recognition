# MultiplyMaskImage
![](images/mul_mask_image.png)

Multiply (bitwise) two mask image into one mask image.

## Subscribing Topic
* `~input/src1` (`sensor_msgs/Image`)
* `~input/src2` (`sensor_msgs/Image`)

  Input mask images.
## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Added mask image.
## Parameters
* `~approximate_sync` (Bool, default: `false`)

  Approximately synchronize `~input/src1` and `~input/src2` if it's true.
