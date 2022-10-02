# draw_tracks.py

![](images/motpy_rect_tracker_node.gif)

Draw rectrangles of TrackArray on a image.

## Subscribing Topics

* `~input` (`sensor_msgs/Image`)

Image topic on which rectangles are drawn.

* `~input/tracks` (`jsk_recognition_msgs/TrackArray`)

TrackArray to draw

## Publishing Topics

* `~output` (`sensor_msgs/Image`)

Image on which rectangles are drawn.

## Parameters

* `~queue_size` (type: `int`, default: `100`)
* `~approximate_sync` (type: `bool`, default: `False`)
* `~slop` (type: `float`, default: `0.1`)

Time Synchronizer Parameters
