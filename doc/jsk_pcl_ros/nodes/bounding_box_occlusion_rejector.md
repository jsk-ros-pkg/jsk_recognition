# BoundingBoxOcclusionRejector
![](images/boundingbox_occlusion_rejector.png)
Rejects bounding boxes which occludes target object.

sample
```
$ roslaunch sample_boundingbox_occlusion_rejector.launch
```

## Publishing Topics
* `~output` (`jsk_recognition_msgs/BoundingBoxArray`)

  Occlusion free candidate bounding boxes.
* `~output/target_image` (`sensor_msgs/Image`)

  Simulated rendered image of target object.
* `~output/candidate_image` (`sensor_msgs/Image`)

  Simulated rendered image of candidate object.

## Subscribing Topics
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  CameraInfo of sensor.
* `~input/target_boxes` (`jsk_recognition_msgs/BoundingBoxArray`)

  BoundingBox array to represent target objects to see.
* `~input/candidate_boxes` (`jsk_recognition_msgs/BoundingBoxArray`)

  BoundingBox array of candidate poses.