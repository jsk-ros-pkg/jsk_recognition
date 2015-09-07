# ROIToRect
Convert camera info with ROI to `geometry_msgs/PolygonStamped`.

## Subscribing Topic
* `~input` (`sensor_msgs/CameraInfo`)

  Input camera info with ROI filled.

## Publishing Topic
* `~output` (`geometry_msgs/PolygonStamped`)

  Output rectangle region.
