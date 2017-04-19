# FlowVelocityThresholding

![](images/flow_velocity_thresholding.gif)

Apply thresholding to optical flow and creates mask image.


## Subscribing Topic

* `~input/flows` (`opencv_apps/FlowArrayStamped`)

  Optical flow.

* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera info to get image size. Subscribed if `~use_camera_info` is `true`.


## Publishing Topic

* `~output` (`sensor_msgs/Image`)

  Mask image.


## Parameters

* `~approximate_sync` (Bool, default: `false`)

  Approximately synchronize inputs if it's true.

* `~queue_size` (Int, default: `100`)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.

* `~use_camera_info` (Bool, default: `true`)

  If `true`, output mask size is got from `~input/camera_info`.

* `~image_height`, `~image_width` (Int)

  Output image height and width.
  Required if `~use_camera_info:=false`.


## Sample

```bash
roslaunch jsk_perception sample_flow_velocity_thresholding.launch
```
