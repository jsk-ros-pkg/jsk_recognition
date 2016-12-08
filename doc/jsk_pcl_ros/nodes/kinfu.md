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