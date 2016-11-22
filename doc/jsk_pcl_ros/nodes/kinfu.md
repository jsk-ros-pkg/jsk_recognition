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

## Advertising Services
* `~save_mesh` (`std_srvs/Empty`)

  Convert tsdf->mesh using marching cubes algorithm, saved as ~/.ros/mesh.ply

* `~get_tsdf` (`jsk_recognition_msgs/GetTsdf`)

  Get tsdf volume, also publish as unknown, empty or occupied. The topics are below.
  ! the occupied topic is too big(if resolution of kinfu is set as default), so may not be shown in rviz or others.

* `~output/unknown` (`sensor_msgs/PointCloud2`)

* `~output/empty` (`sensor_msgs/PointCloud2`)

* `~output/occupied` (`sensor_msgs/PointCloud2`)

