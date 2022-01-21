# motpy_rect_tracker_node.py

Multi Object Tracking Node with [motpy](https://github.com/wmuron/motpy) implementation of [SORT](https://arxiv.org/abs/1602.00763) algorithm.

![](images/motpy_rect_tracker_node.gif)

## Subscribing Topics

* `~input` (`sensor_msgs/Image`)

Panorama image topic.

* `~input/rects` (`jsk_recognition_msgs/RectArray`)

RectArray from object detection with the panorama image topic

* `~input/class` (`jsk_recognition_msgs/ClassificationResult`)

ClassificationResult from object detection with the panorama image topic

## Publishing Topics

* `~output/tracks` (`jsk_recognition_msgs/TrackArray`)

TrackArray of each tracked objects.

* `~output/viz` (`sensor_msgs/Image`)

Visualization image.

## Parameters

* `~dt` (type: `float`)
* `~min_iou` (type: `float`, default: `0.1`)
* `~multi_match_min_iou` (type: `float`, default: `0.1`)
* `~min_steps_alive` (type: `int`, default: `3`)

Parameters for multi object tracking

* `~target_labels` (type: `list of string`, default: `[]`)

List of labels to track.

* `~queue_size` (type: `int`, default: `100`)
* `~approximate_sync` (type: `bool`, default: `False`)
* `~slop` (type: `float`, default: `0.1`)

Parameters for TimeSynchronizer.

## Sample

```bash
roslaunch jsk_perception sample_motpy_rect_tracker_node.launch
```
