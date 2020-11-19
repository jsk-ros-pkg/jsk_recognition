# xyz_to_screenpoint.py

## What Is This

Convert (x, y, z) 3-D coordinate to (u, v) coordinate on a image using camerainfo of sensor.


## Subscribing Topic

* `~input` (`geometry_msgs/PointStamped`)

  Input point to represent (x, y, z) 3-D coordinate.

* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  CameraInfo of sensor.


## Publishing Topic

* `~output` (`geometry_msgs/PointStamped`)

  Output point to represent (u, v) image coordinate. Only x and y fileds are used.
  The header frame_id uses the information of `~input/camera_info`.


## Parameters

None.


## Sample

```bash
roslaunch jsk_pcl_ros_utils sample_xyz_to_screenpoint.launch
```
