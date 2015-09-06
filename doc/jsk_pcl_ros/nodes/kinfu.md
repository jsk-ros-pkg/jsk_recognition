# Kinfu
Publishes camera pose using pcl/KinfuLS

## Subscribing Topics
* `~input/info` (`sensor_msgs/CameraInfo`)

  Intrinsic camera parameter of depth image
* `~input/depth` (`sensor_msgs/Image`)

  Depth image in m and the format should be 32FC1
* `~input/color` (`sensor_msgs/Image`)

  RGB color image (not yet used)

## Publishing Topics
* `~output` (`geometry_msgs/PoseStamped`)

  Pose of camera
* `~output/cloud` (`sensor_msgs/PointCloud2`)

  Registered scene pointcloud
