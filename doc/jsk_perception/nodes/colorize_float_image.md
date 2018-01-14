# ColorizeFloatImage
![](images/heightmap_converter.png)

Colorize float image with heatmap gradation.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Float(32FC[1-4]) image.
## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  RGB8 image with heatmap gradation.
## Parameters
* `~channel` (int, default: 0)

  Choose channel to colorize. If value is default, first channel is selected.

## Sample

```bash
roslaunch jsk_perception sample_color_float_image.launch
```
