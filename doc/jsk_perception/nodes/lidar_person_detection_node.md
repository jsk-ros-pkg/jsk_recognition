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

* `~output/markers` (`visualization_msgs/MarkerArray`)

  MakerArray of detected people.

## Parameters

* `~queue_size` (`Int`, default: `1`)

  Input queue size.

* `~conf_thresh` (`Double`, default: `0.8`)

  Threshold for confidence.

* `~weight_file` (`String`, required)

  Threshold for confidence.

* `~detector_model` (`String`, default: `DR-SPAAM`)

  Detector model. Current only `DR-SPAAM` is supported.

* `~stride` (`Int`, default: `1`)

  Use this to skip laser points.

* `~panoramic_scan` (`Bool`, default: `false`)

  Use this to skip laser points.

* `~base_link` (`String`, default: `None` optional)

  If this value is specified, markers are published in `~base_link` frame.

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
