# PointCloudLocalization
![](images/pointcloud_localization.png)

Localize 6d pose of robot using ICP registration of pointcloud.
It publishes tf transformation from gloabl frame to odometory frame like acml does.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud to align.

## Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Concatenated pointcloud.

## Parameters
* `~tf_rate` (Double, default: `20.0`)

  Frequency to publish tf transformations.
* `~cloud_rate` (Double, default: `10.0`)

  Frequency to publish `~output` topic.
* `~leaf_size` (Double, default: `0.01`)

  Resolution of voxel grid downsampling.
* `~use_normal` (Bool, default: `false`)

  Support normal.

## Using Services
* `~icp_align` (`jsk_pcl_ros/ICPAlign`)

  ICP service to align pointcloud

## Advertising Services
* `~localize` (`std_srvs/Empty`)

  Run localization
* `~update_offset` (`jsk_pcl_ros/UpdateOffset`)

  Update transformation between odom frame and global frame manuaaly.
  Currently no tf is resolved.
