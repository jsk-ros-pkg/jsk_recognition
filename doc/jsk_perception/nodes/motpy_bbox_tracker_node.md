# motpy_bbox_tracker_node.py

Bounding Box Multi Object Tracking Node with [motpy](https://github.com/wmuron/motpy) implementation of [SORT](https://arxiv.org/abs/1602.00763) algorithm.

![](images/motpy_bbox_tracker_node.gif)

## Subscribing Topics

* `~input/rects` (`jsk_recognition_msgs/BoundingBoxArray`)

BoundingBoxArray to track.

## Publishing Topics

* `~output/bbox_array` (`jsk_recognition_msgs/BoundingBoxArray`)

BoundingBoxArray of each tracked objects. `label` field of each Bounding Box is filled with track ID.

* `~output/obj_array` (`jsk_recognition_msgs/ObjectArray`)

ObjectArray of each tracked objects.

## Parameters

* `~dt` (type: `float`)
* `~min_iou` (type: `float`, default: `0.1`)
* `~multi_match_min_iou` (type: `float`, default: `0.1`)
* `~min_steps_alive` (type: `int`, default: `3`)

Parameters for multi object tracking

* `~target_labels` (type: `list of string`, default: `[]`)

List of labels to track.

## Sample

```bash
roslaunch jsk_perception sample_motpy_bbox_tracker_node.launch
```
