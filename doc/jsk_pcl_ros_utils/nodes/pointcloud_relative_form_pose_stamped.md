# PointCloudRelativeFromPoseStamped

Transform pointcloud relative from the specified pose stamped.
It is useful for preprocessing of registration to detect initial pose.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`)
* `~input/pose` (`geometry_msgs/PoseStamped`)

## Publishing Topics
* `~output` (`sensor_msgs/PointCloud2`)
