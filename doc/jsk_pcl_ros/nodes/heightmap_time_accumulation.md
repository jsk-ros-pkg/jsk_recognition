# HeightmapTimeAccumulation
![](images/heightmap_time_accumulation.png)


Accumulate heightmap in time series and construct a new heightmap.

## Subscription Topic
* `~input` (`sensor_msgs/Image`)

  Input new heightmap(t=k).
* `~input/prev_pointcloud` (`sensor_msgs/PointCloud2`)

  Accumulated heightmap represented in pointcloud from 0 to k-1 step.
* `~input/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Accumulated heightmap.

* `~output/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.

## Advertising Service
* `~seset` (`std_srvs/Empty`)

  Reset heightmap cache.
