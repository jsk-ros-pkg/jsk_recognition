# RectArrayToMaskImage
Convert array of rectangle (`jsk_recognition_msgs/PolygonArray`) into mask image (`sensor_msgs/Image`)

We expect it will be used with image_view2.

## Subscribing Topic
* `~input` (`jsk_recognition_msgs/PolygonArray`)

  PolygonArray to represent rectangle region of image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Original camera info.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Mask image.
