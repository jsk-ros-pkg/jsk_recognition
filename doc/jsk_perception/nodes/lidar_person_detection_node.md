# lidar_person_detection_node.py

![](images/lidar_person_detection_node.gif)

Detect person from lidar sensor.

For more detail, please see the thesis `DR-SPAAM: A Spatial-Attention and Auto-regressive Model for Person Detection in 2D Range Data`.

In order to use this feature, you need to install `pytorch <https://pytorch.org/get-started/locally/>`_ (pytorch >= 1.4.0 is recommended).


## Subscribing Topic

* `~input` (`sensor_msgs/LaserScan`)

  Input laser scan.


## Publishing Topic

* `~output` (`geometry_msgs/PoseArray`)

  Position of detected people.

  Based on the result of tracking, the direction of movement is the x direction.

* `~output/markers` (`visualization_msgs/MarkerArray`)

  MakerArray of detected people.

  The color of the marker is determined based on the tracking result.

## Parameters

* `~queue_size` (`Int`, default: `1`)

  Input queue size.

* `~conf_thresh` (`Double`, default: `0.8`)

  Threshold for confidence.

* `~weight_file` (`String`, required)

  Trained model's weight file path.

* `~detector_model` (`String`, default: `DR-SPAAM`)

  Detector model. Current only `DR-SPAAM` is supported.

* `~stride` (`Int`, default: `1`)

  Use this to skip laser points.

* `~panoramic_scan` (`Bool`, default: `false`)

  Set to true if the scan covers 360 degree.

* `~gpu` (`Int`, default: `-1`)

  Index of gpu used for prediction. Set `-1` for using CPU.

* `~max_distance` (`Double`, default: `0.5`)

  Threshold for tracking max distance.

  If the position in the previous frame is farther than this distance, it will be excluded from the tracking candidates.

* `~n_previous` (`Int`, default: `10`)

  Determine the moving direction from the previous position and the current position.

* `~map_link` (`String`, default: `None` optional)

  If this value is specified, markers are published in `~map_link` frame.

* `~duration_timeout` (`Double`, default: `0.05`)

  Duration of timeout for lookup transform in case of specifying `~map_link`.

* `~color_alpha` (`Double`, default: `0.8`)

  Alpha value of visualization marker.

* `~people_height` (`Double`, default: `1.6`)

  Height of visualization marker.

* `~people_head_radius` (`Double`, default: `0.3`)

  Head radius of visualization marker.

* `~people_body_radius` (`Double`, default: `0.3`)

  Body radius of visualization marker.

## Sample

```bash
roslaunch jsk_perception sample_lidar_person_detection.launch
```
