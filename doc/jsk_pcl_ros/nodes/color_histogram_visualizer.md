ColorHistogramVisualizer
========================

![color_1d](https://user-images.githubusercontent.com/1901008/27892782-924c6ed0-623c-11e7-9219-28a849cfe8e0.png)
![histogram_hue_sat](https://user-images.githubusercontent.com/1901008/26914207-e118a968-4c5a-11e7-862f-705895aeee77.gif)

Visualize 1D / 2D color histogram

## Run

``` bash
rosrun jsk_pcl_ros color_histogram_visualizer.py
```

## Subscribing Topics

* `~input` (`jsk_recognition_msgs/ColorHistogram`)

    Input color histogram

* `~input/array` (`jsk_recognition_msgs/ColorHistogramArray`)

    Input color histogram array

## Publishing Topics

* `~output` (`sensor_msgs/Image`)

    Image of histogram bar graph

## Parameters

* `~histogram_policy`:

    See [ColorHistogram](color_histogram.md)

* `~histogram_index` (`Int`, default: `0`):

    Index of input array to show histogram
    (Valid only when input is `ColorHistogramArray`)

* `~histogram_scale` (`Double`, default: `1.0`):

    Scale factor for each color histogram
    (Scale up when bar is too short to visualize)
