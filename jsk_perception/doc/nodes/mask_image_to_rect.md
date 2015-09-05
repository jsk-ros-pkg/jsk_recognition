# MaskImageToRect
Convert a mask image into geometry_msgs::PolygonStamped.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask image.

## Publishing Topic
* `~output` (`geometry_msgs/PolygonStamped`)

  PolygonStamped message which only contains two points. Minimum point and Maximum point to represent bounding box in image.

