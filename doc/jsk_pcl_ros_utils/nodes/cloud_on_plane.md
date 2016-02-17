CloudOnPlane
===========

Publishes true when a pointcloud is on a polygon.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`)
* `~input/polygon` (`jsk_recognition_msgs/PolygonArray`)

  Input pointcloud and polygons.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/BoolStamped`)

  True if distance between pointcloud and polygon is smaller than
  `~distance_thr` for `~buf_size` frames.

## Parameters
* `~distance_thr` (default: `0.05`)

  Distance threshold between pointcloud and polygon.
* `~buf_size` (default: `2`)

  CloudOnPlane only returns true if all the recent `~buf_size` results is true.
