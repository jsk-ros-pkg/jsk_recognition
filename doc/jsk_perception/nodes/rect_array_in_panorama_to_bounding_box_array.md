# rect_array_in_panorama_to_bounding_box_array.py
![](images/rect_array_in_panorama_to_bounding_box_array.gif)

This node calculates distances to RectArray in a panorama image with FOV of the image, heights of rects and heights of each labels with the equation `(object height [m]) = (distance [m]) * (object height in panorama image [rad])`, and publish BoundingBoxArray.

Sample is `sample_rect_array_in_panorama_to_bounding_box_array.launch`

## Subscribing Topics

* `~panorama_image` (`sensor_msgs/Image`)

Panorama image topic.

* `~panorama_info` (`sensor_msgs/PanoramaInfo`)

Panorama info topic.

* `~input_class` (`jsk_recognition_msgs/ClassificationResult`)

ClassificationResult from object detection with the panorama image topic

* `~input_rects` (`jsk_recognition_msgs/RectArray`)

RectArray from object detection with the panorama image topic

## Publishing Topics

* `~bbox_array` (`jsk_recognition_msgs/BoundingBoxArray`)

BoundingBoxArray of each object of output of object detection.

## Parameters

* `~frame_fixed` (string, default: `fixed_frame`)

The frame_id of BoundingBoxArray. Assumed to be a fixed frame.

* `~dimensions_labels` (dict of string to lists of float values, default: `{}`)

Dimensions for each labels. These are used for BoundingBoxArray and also calculation of distance.

e.g. `{'person':[0.5,0.5,1.5]}`

* `~duration_timeout` (float, default: `0.05)

Duration of timeout for lookup transform

## Sample

```bash
roslaunch jsk_perception sample_rect_array_in_panorama_to_bounding_box_array.launch
```
