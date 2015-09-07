# HeightmapToPointCloud
![](images/heightmap_to_pointcloud.png)

Convert a heightmapt to poincloud.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input heightmap.
* `~input/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.

## Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Output pointcloud.

* `~output/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.