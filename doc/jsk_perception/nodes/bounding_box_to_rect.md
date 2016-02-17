# BoundingBoxToRect
![](images/bounding_box_to_rect.png)

Convert `jsk_recognition_msgs/BoundingBoxArray` to `jsk_recognition_msgs/RectArray`.

## Subscribing Topics
* `~input` (`jsk_recognition_msgs/BoundingBoxArray`)

  Input bounding boxes.
* `~input/info` (`sensor_msgs/CameraInfo`)

  CameraInfo to project bounding boxes.

* `~internal` (`jsk_recognition_msgs/BoundingBoxArrayWithCameraInfo`)

  Internal topic to synchronize timestamp of `~input` and `~input/info`.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/RectArray`)

  Projected 2-D bounding box.

* `~internal` (`jsk_recognition_msgs/BoundingBoxArrayWithCameraInfo`)
