# PolygonMagnifier
Magnify polygons by specified length.

## Subscribing Topic
* `~input` (`jsk_recognition_msgs/PolygonArray`)

  Input polygons

## Publishing Topic
* `~output` (`jsk_recognition_msgs/PolygonArray`)

  Output magnified polygons

## Parameters
* `~magnify_distance` (Double, default: `0.2`)

  Length to scale polygon
