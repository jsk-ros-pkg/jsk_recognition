# jsk_pcl_ros

## Introduction
jsk\_pcl\_ros is a package to provide some programs using [pcl](http://pointclouds.org).

This package provides some programs as nodelet.

## types
jsk\_pcl\_ros provides several message types.
### ClusterPointIndices.msg
```
Header header
pcl_msgs/PointIndices[] cluster_indices
```
ClusterPointIndices is used to represent segmentation result.
Simply put, ClusterPointIndices is a list of PointIndices.

### ModelCoefficientsArray
```
Header header
pcl_msgs/ModelCoefficients[] coefficients
```
ModelCoefficientsArray is used to represent coefficients of model
for each segmented clusters.
Simply put, ModelCoefficientsArray is a list of ModelCoefficients.

### PolygonArray
```
Header header
geometry_msgs/PolygonStamped[] polygons
```
PolygonArray is a list of PolygonStamped.

You can use [jsk\_rviz\_plugins](https://github.com/jsk-ros-pkg/jsk_visualization) to visualize PolygonArray in rviz.

### BoundingBox
```
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions #x, y and z
```
BoundingBox represent a oriented bounding box. `dimensions` mean the
size of bounding box.

### BoundingBoxArray
```
Header header
BoundingBox[] boxes
```
BoundingBoxArray is a list of BoundingBox.
You can use [jsk\_rviz\_plugins](https://github.com/jsk-ros-pkg/jsk_visualization) to visualize BoungingBoxArray in rviz.

### TimeRange
```
Header header
time start
time end
```
Represent range of time.

## nodelets
### jsk\_pcl/ParticleFilterTracking
#### What Is This

This nodelet will track the target pointcloud which you set in rviz.

#### Sample

run the below command.

```
roslaunch jsk_pcl_ros tracking_groovy.launch # (When use groovy)
roslaunch jsk_pcl_ros tracking_hydro.launch  #(When use hydro)
```

Push the "Select" button at the top bar , drag and surround the target poincloud which you want to track in the rectangle area.Then, finally, push the "SelectPointCloudPublishActoin" button at SelectPointCloudPublishAction Panel. The tracker will start tracking the target.

### jsk\_pcl/ResizePointsPublisher
#### What is this
ResizePointsPublisher resizes PointCloud generated from depth images. It keeps *organized* pointcloud. For example you can create QVGA pointcloud from VGA pointcloud of kinect like sensors.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):
Input PointCloud. The input should be organized pointcloud.
#### Publishing Topics.
* `~output` (`sensor_msgs/PointCloud2`):
Output PointCloud. The output will be organized.

#### Parameters
* `~step_x`, `~step_y` (Double, default: `2`):
Bining step when resizing pointcloud.
* `~not_use_rgb` (Boolean, default: `false`):
If you want to resize pointcloud without RGB fields, you need to set this parameter to True.

### jsk\_pcl/PointcloudScreenpoint
#### What is this
This is a nodelet to convert (u, v) coordinate on a image to 3-D point.
It retrieves 3-D environment as pointcloud.

#### Subscribing Topics
* `~points` (`sensor_msgs/PointCloud2`):

Input pointcloud to represent 3-D environment it should be organized.
* `~point` (`geometry_msgs/PointStamped`):

Input point to represent (u, v) image coordinate and this topic is enabled only if `~use_point` parameter is set `True`.
Only x and y fileds are used and the header frame_id is ignored.
If `~use_sync` parameter is set `True`, `~points` and `~point` are synchronized.

* `~polygon` (`geometry_msgs/PolygonStamped`):

Input rectangular region on image local coordinates and this topic is enabled only if `~use_rect` parameter is set `True`.
Only x and y fields are used and the header frame_id is ignored.
And the region should be rectangular.
If `~use_sync` parameter is set `True`,

* `~point_array` (`sensor_msgs/PointCloud2`):

Input points to represent series of (u, v) image coordinate and this
topic is enabled only if `~use_point_array` parameter is set `True`.
Only x and y fields are used and the header frame_id is ignored.
If `~use_sync` parameter is set `True`, `~point_array` and `~point` are
synchronized.

#### Publishing Topics
* `~output_point` (`geometry_msgs/PointStamped`):
The topic to be used to publish one point as a result of screenpoint.

* `~output` (`sensor_msgs/PointCloud`):
The topic to be used to publish series of points as a result of screenpoint.

#### Advertising Servicies
* `~screen_to_point` (`jsk_pcl_ros::TransformScreenpoint`)
ROS Service interface to convert (u, v) image coordinate into 3-D point.

The definition of `jsk_pcl_ros::TransformScreenpoint` is:
```
# screen point
float32 x
float32 y
---
# position in actual world
std_msgs/Header header
geometry_msgs/Point point
geometry_msgs/Vector3 vector
```

With int this service, the latest pointcloud acquired by `~points` is used to convert (u, v) into 3-D point.

#### Parameters
* `~use_sync` (Boolean, default: `False`):
If this parameter is set to `True`, the timestamps of 3-D pointcloud and the target point/rectangle/point array are synchronized.

* `~queue_size` (Integer, default: `1`):
Queue length of subscribing topics.

* `~crop_size` (Integer, default: `10`):
The size of approximate region if `~points` pointcloud has nan holes.

* `~use_rect` (Boolean, default: `False`):
Enable `~polygon` topic.

* `~use_point` (Boolean, default: `False`):
Enable `~point` topic.

* `~use_point_array` (Boolean, default: `False`):
Enable `~point_array` topic.

* `~publish_points` (Boolean, default: `False`):
Publish result of screenpoint to `~output` topic.
* `~publish_point` (Boolean, default: `False`):
Publish result of screenpoint to `~output_point` topic.

### jsk\_pcl/TiltLaserListener
#### What is this
Listen to the joint_states of tilt/spindle laser and publish time range to scane full 3-D space.
You can choose several types of tilt/spindle lasers such as tilt-laser of PR2, infinite spindle laser of multisense.

#### Subscribing Topics
* `~input`(`sensor_msgs/JointState`): Joint angles of laser actuator.

#### Publishing Topics
* `~output` (`jsk_pcl_ros/TimeRange`): Time range to scan 3-D space.
* `~output_cloud` (`sensor_msgs/PointCloud2`): Assembled pointcloud according to time range
of `~output`. this require `~assemble_scans2` service of [laser_assembler](http://wiki.ros.org/laser_assembler).

#### Using Services
* `~assemble_scans2` (`laser_assembler/AssembleScans2`): A service to build 3-D pointcloud from scan pointcloud.
It should be remapped to `assemble_scans2` service of [laser_assembler](http://wiki.ros.org/laser_assembler).

#### Parameters
* `~use_laser_assembler` (Boolean, default: `False`): Enable `~output_cloud` and `~assemble_scans2`.
* `~joint_name` (String, **required**): Joint name of actuator to rotate laser.
* `~laser_type` (String, default: `tilt_half_down`): type of rotating laser. You can choose one of the types:
1. `tilt`: A mode for tilting laser. In this mode, TiltLaserListener assumes the motor to be moved from minimum
joint angle to maximum joint angle over again. TiltLaserListener publishes the minimum and latest time range to
move tilting laser from minimum joint angle to maximim joint angle.
2. `tilt_half_down`:
In this mode, TiltLaserListener publishes time range from maximum joint angle to minimum joint angle.
3. `tilt_half_up`:
In this mode, TiltLaserListener publishes time range from minimum joint angle to maximum joint angle like `tilt_half_down`.
4. `infinite_spindle`: Infinite spindle laser. TiltLaserListener publishes time range to rotate laser 360 degrees.
5. `infinite_spindle_half`: Infinite spindle laser, but most of laser has over 180 degrees range of field.
Therefore we don't need to rotate laser 360 degrees to scan 3-D space, just 180 degree rotation is required.
In this mode, TiltLaserListener publishes time range a time range of 180 degree rotation.

### jsk\_pcl/DepthImageCreator
#### What is this
Create *organized* pointcloud from non-organized pointcloud.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

The input pointcloud to be reconstructed as organized pointcloud.

* `~info` (`sensor_msgs/CameraInfo`):

Put a simulated camera according to `~info` and generate organized pointcloud.


#### Publishing Topics
* `~output` (`sensor_msgs/Image`):
Publish organized pointcloud as depth image.

* `~output_cloud` (`sensor_msgs/PointCloud2`)
organized pointcloud.

* `~output_disp` (`sensor_msgs/DisparityImage`)
Publish organized pointcloud as disparity image.

#### Parameters
* `~scale_depth` (Double, default: `1.0`)
scale depth value.
* `~use_fixed_transform` (Boolean, default: `False`):
* `~translation` (Array of double, default: `[0, 0, 0]`)
* `~rotation` (Array of double, default: `[0, 0, 0, 1]`)
If `~use_fixed_transform` is set to `True`,
transformation between `~input` and `~info` is not resolved via tf
but fixed transformation is used according to `~rotation` and `translation`.
* `~use_asynchronous` (Boolean, default: `False`)
Do not synchronize `~input` and `~info` if this parameter is set to `True`.
* `~use_approximate` (Boolean, default: `False`)
Synchronize `~input` and `~info` approximately if this parameter is set to `True`.
* `~info_throttle` (Integer, default: `0`)
The number of `~info` messages to skip to generate depth image.
* `~max_queue_size` (integer, default: `3`):
Queue length of topics.

### jsk\_pcl/EuclideanClustering
![](images/euclidean_segmentation.png)
#### What Is This
Segment pointcloud based euclidean metrics, which is based on `pcl::EuclideanClusterExtraction`.
This nodelet has topic interface and service interface.

The result of clustering is published as `jsk_pcl_ros/ClusterPointIndices`.

If the number of the cluster is not changed across different frames, `EuclideanClustering`
tries to track the segment.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):
input pointcloud.

#### Publishing Topics
* `~output` (`jsk_pcl_ros/ClusterPointIndices`):
Result of clustering.

* `~cluster_num` (`jsk_pcl_ros/Int32Stamped`):
The number of clusters.

#### Advertising Services
* `~euclidean_clustering` (`jsk_pcl_ros/EuclideanSegment`):
Service interface to segment clusters.

```
sensor_msgs/PointCloud2 input
float32 tolerance
---
sensor_msgs/PointCloud2[] output
```

#### Parameters
* `~tolerance` (Double, default: `0.02`):
Max distance for the points to be regarded as same cluster.
* `~label_tracking_tolerance` (Double, default: `0.2`)
Max distance to track the cluster between different frames.
* `~max_size` (Integer, default: `25000`)
The maximum number of the points of one cluster.
* `~min_size` (Integer, default: `20`)
The minimum number of the points of one cluster.

#### Sample
Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros euclidean_segmentation.launch
```

### jsk\_pcl/ClusterPointIndicesDecomposer
![](images/bounding_box.png)
#### What is this
Decompose `jsk_pcl_ros/ClusterPointIndices` into array of topics of `sensor_msgs/PointCloud` like `~output00`, `~output01` and so on.
It also publishes tf of centroids of each cluster and oriented bounding box of them. The direction of the bounding box are aligned on to the nearest planes if available.

#### Subscribing topics
* `~input` (`sensor_msgs/PointCloud2`):
Input pointcloud.

* `~target` (`jsk_pcl_ros/ClusterPointIndices`):
Input set of indices to represent clusters.

* `~align_planes` (`jsk_pcl_ros/PolygonArray`):
* `~align_planes_coefficients` (`jsk_pcl_ros/ModelCoefficientsArray`):
The planes for bounding box to be aligned on.

#### Publishing topics
* `~output%02d` (`sensor_msgs/PointCloud2`):
Series of topics for each pointcloud cluster.
* `~debug_output` (`sensor_msgs/PointCloud2`):
Concatenate all the clusters into one pointcloud and colorize each cluster to see the result of segmentation.
* `~boxes` (`jsk_pcl_ros/BoundingBoxArray`):
Array of oriented bounding box for each segmented cluster.

#### Parameters
* `~publish_tf` (Boolean, default: `True`):
Toggle tf publishing.
* `~publish_clouds` (Boolean, default: `True`):
Toggle `~output%02d` topics.
* `~align_boxes` (Boolean, default: `False`):
If this parameter is set to `True`, `~align_planes` and `~align_planes_coefficients` are enabled.
* `~use_pca` (Boolean, default: `False`):
Run PCA algorithm on each cluster to estimate x and y direction.

### jsk\_pcl/ClusterPointIndicesDecomposerZAxis
#### What Is This
This nodelet is almost same to jsk\_pcl/ClusterPointIndicesDecomposer, however it always sort clusters in z direction.

### jsk\_pcl/CentroidPublisher
#### What Is This

This nodelet will subscribe the sensor\_msgs::PointCloud2, calculate its centroid  and boardcast the tf whose parent is cloud headers frame\_id and whose child is the new centroid frame_id.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`): input pointcloud.
#### Publishing Topics
* `/tf`: Publish tf of the centroid of the input pointcloud.
#### Parameters
* `~frame` (String, required): frame_id of centroid tf

#### Sample
Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros centroid_publisher.launch
```

### jsk\_pcl/VoxelGridDownsampleManager
### jsk\_pcl/VoxelGridDownsampleDecoder
### jsk\_pcl/Snapit
#### What Is This


This nodelet will snap the plane to the real world pointcloud.
Move the interactive marker and the snapped plane will follow the movement.

#### Sample

Plug the depth sensor which can be launched by openni.launch and run the below command.


```
roslaunch jsk_pcl_ros snapit_sample.launch
```

### jsk\_pcl/KeypointsPublisher
#### What Is This

This nodelet will calculate the NURF keypoints and publish.

#### Sample

Plug the depth sensor which can be launched by openni.launch and run the below command.


```
roslaunch jsk_pcl_ros keypoints_publisher.launch
```
### jsk\_pcl/HintedPlaneDetector
#### What Is This


This nodelet will snap the plane to the real world pointcloud.
Move the interactive marker and the snapped plane will follow the movement.

#### Sample

Plug the depth sensor which can be launched by openni.launch and run the below command.


```
roslaunch jsk_pcl_ros hinted_plane_detector_sample.launch
```

### jsk\_pcl/OctreeChangeDetector
#### What Is This

This nodelet will publish the difference of sequential pointcloud. You can get the newly generated pointclouds.

Difference with pcl_ros/SegmentDifference refer https://github.com/jsk-ros-pkg/jsk_recognition/pull/67

#### Sample

Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros octree_change_detector.launch
```

#### Speed

### jsk\_pcl/TfTransformCloud
#### What Is This

This nodelet will republish the pointcloud which is transformed with the designated frame_id.

#### Topics
* Input
  * `~input` (`sensor_msgs/PointCloud2`): input pointcloud
* Output
  * `~output` (`sensor_msgs/PointCloud2`): output pointcloud.

#### Parameters
* `~target_frame_id` (string): The frame_id to transform pointcloud.

#### Sample
Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros tf_transform_cloud.launch
```

## To Test Some Samples

Please be careful about the nodelet manager name when execute some sample launches.

Because the nodelet manager name is different between groovy version and hydro version in openni.launch,
you have to replace the nodelet manager name when use in groovy as below.

From

```
/camera_nodelet_manager
```

To

```
/camera/camera_nodelet_manager
```
