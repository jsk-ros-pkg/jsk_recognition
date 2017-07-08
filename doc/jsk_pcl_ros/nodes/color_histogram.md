ColorHistogram
==============

![hue_sat](https://user-images.githubusercontent.com/1901008/26964249-a3a89382-4d2b-11e7-8ce1-c4a12f57ee06.gif)

Compute color histogram for each cluster point indices.

## Subscribing Topics

* `~input` (`sensor_msgs/PointCloud2`)

    Input point cloud

* `~input/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Input point indices

## Publishing Topics

* `~output` (`jsk_recognition_msgs/ColorHistogramArray`)

    Color histogram for each point indices
    Each histogram contains either Hue, Saturation (as 1-d vector) or both of two values (as 2-d matrix). (configurable by parameter `~histogram_policy`)
    Bin size is also configurable by parameter `~bin_size`.

## Parameters

* `~queue_size` (`Int`, default: `100`)

    Queue size for message synchronization

* `~bin_size` (`Int`, default: `100`)

    Bin size for histogram  
    If a parameter `~histogram_policy` is set as `HUE_AND_SATURATION`, actual vector length of each histogram is `bin_size * bin_size`, otherwise `bin_size`.

* `~histogram_policy` (`Enum[Int]`, default: `HUE_AND_SATURATION`)

    Policy for histogram values to accumulate

    - 0: `HUE`
        - Use hue only
    - 1: `SATURATION`
        - Use saturation only
    - 2: `HUE_AND_SATURATION`
        - Use both hue and saturation as 2-d matrix

## How to visualize

See [ColorHistogramVisualizer](color_histogram_visualizer.md)

## Sample

```bash
roslaunch jsk_pcl_ros sample_color_histogram.launch
```
