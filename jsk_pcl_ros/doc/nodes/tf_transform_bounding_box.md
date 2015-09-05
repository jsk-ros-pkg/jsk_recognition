# TfTransformBoundingBox
This nodelet will republish bounding box which is transformed with the designated frame_id.

## Subscribing Topics
* `~input` (`jsk_recognition_msgs/BoundingBox`)

  input bounding box.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/BoundingBox`)

  output bounding box.

## Parameters
* `~target_frame_id` (string): The frame_id to transform pointcloud.
* `~use_latest_tf` (Bool, default: `false`)

  If this parameter is true, ignore timestamp of tf to transform pointcloud.
* `~tf_queue_size` (Int, default: `10`)

  Queue size of tf message filter to synchronize tf and `~input` topic.

## Depth Camera Calibration(Kinect,Xtion,Primesense)
![](images/depth_calibration.png)