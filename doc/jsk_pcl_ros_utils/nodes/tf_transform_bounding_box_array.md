# TfTransformBoundingBoxArray

![](images/tf_transform_bounding_box_array.png)

This nodelet will republish bounding box array which is transformed with the designated frame_id.

## Subscribing Topics
* `~input` (`jsk_recognition_msgs/BoundingBoxArray`)

  input bounding box array.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/BoundingBoxArray`)

  output bounding box array.

## Parameters
* `~target_frame_id` (string, required)

  The frame_id to transform bounding box array.

* `~use_latest_tf` (Bool, default: `false`)

  If this parameter is true, ignore timestamp of tf to transform bounding box array.

* `~tf_queue_size` (Int, default: `10`)

  Queue size of tf message filter to synchronize tf and `~input` topic.

## Sample

```bash
roslaunch jsk_pcl_ros_utils sample_tf_transform_bounding_box_array.launch
```
