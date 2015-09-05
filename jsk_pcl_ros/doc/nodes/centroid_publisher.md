# CentroidPublisher
## What Is This

This nodelet will subscribe the sensor\_msgs::PointCloud2, calculate its centroid and
publish tf, `geometry_msgs/PoseStamped` and `geometry_msgs/PointStamped`.
The nodelet boardcast the tf whose parent is cloud headers frame\_id and whose child is the new centroid frame_id.

## Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.
## Publishing Topics
* `/tf`

   Publish tf of the centroid of the input pointcloud.
* `~output/pose` (`geometry_msgs/PoseStamped`)

   centroid of the pointcloud as `geometry_msgs/PoseStamped`.
* `~output/point` (`geometry_msgs/PointStamped`)

   centroid of the pointcloud as `geometry_msgs/PointStamped`.
## Parameters
* `~frame` (String, required):

   frame_id of centroid tf
* `~publish_tf` (Boolean, default: `False`)
  Set this parameter true in order to publish tf frame.

## Sample
Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros centroid_publisher.launch
```
