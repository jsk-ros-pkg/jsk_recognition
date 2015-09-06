# HeightmapConverter
![](images/heightmap_converter.png)

Convert a pointcloud(`sensor_msgs/PointCloud2`) into heightmap representation (`sensor_msgs/Image`).

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  fields of the image is `CV_32FC1(float)` and if a pixel is not observed, it is filled by `-FLT_MAX`.

* `~output/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.
## Parameters
* `~resolution_x` (Integer, default: `400`)
* `~resolution_y` (Integer, default: `400`)

  Resolution of height map

* `~min_x` (Double, default: `-2.0`)
* `~max_x` (Double, default: `2.0`)
* `~min_y` (Double, default: `-2.0`)
* `~max_y` (Double, default: `2.0`)

  Minimum and maximum value of heightmap dimension.
