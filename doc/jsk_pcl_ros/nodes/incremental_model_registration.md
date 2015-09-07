# IncrementalModelRegistration
## What Is This
![](images/incremental_model_registration.png)

Build a full-model from sequential captured data.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. RGB field is required.
* `~input/pose` (`geometry_msgs/PoseStamped`)

  Initial pose to estimate acculate pose of the pointcloud.
* `~input/indices` (`pcl_msgs/PointIndices`)

  Indices to mask object in `~input` pointcloud.

## Publishing Topic
* `~output/non_registered` (`sensor_msgs/PointCloud2`)

  Pointcloud just concatenated according to `~input/pose`

* `~output/registered` (`sensor_msgs/PointCloud2`)

  Pointcloud refined by ICP.
## Using Services
* `~icp_service` (`jsk_pcl_ros/ICPAlign`)

  ICP service interface to refine model.
