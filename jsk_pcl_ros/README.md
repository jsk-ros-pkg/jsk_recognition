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
### jsk\_pcl/HintedStickFInder
![](images/hinted_stick_finder.png)

Detect a stick from pointcloud and line in 2-D image as hiint.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameter where hint line is defined

* `~input/hint/line` (`geometry_msgs/PolygonStamped`)

  Hint line described in 2-D image.

#### Publishing Topic
* `~debug/line_filtered_indices` (`pcl_msgs/PointIndices`)

  Indices of input pointcloud which is filtered by hint line.

* `~debug/line_filtered_normal` (`sensor_msgs/PointCloud`)

  Normal pointcloud of filtered pointcloud.

* `~debug/cylinder_marker` (`visualization_marker/Marker`)

  Marker topic to visualize detected stick

* `~debug/cylinder_pose` (`geometry_msgs/PoseStamped`)

  Pose of detected stick.

### jsk\_pcl/RGBColorFilter
Filter pointcloud based on RGB range.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. rgb field is required.

* `~indices` (`pcl_msgs/PointIndices`)

  Indices of pointcloud. only available if `~use_indices` is true.

#### Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Filtered pointcloud.

#### Parameters
* `~r_max` (Integer, default: `255`)
* `~r_min` (Integer, default: `0`)
* `~g_max` (Integer, default: `255`)
* `~g_min` (Integer, default: `0`)
* `~b_max` (Integer, default: `255`)
* `~b_min` (Integer, default: `0`)

  Color range to filter.

### jsk\_pcl/HSIColorFilter
![](images/hsi_color_filter.png)

Filter pointcloud based on HSI range.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. rgb field is required.

* `~indices` (`pcl_msgs/PointIndices`)

  Indices of pointcloud. only available if `~use_indices` is true.

#### Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Filtered pointcloud.

#### Parameters
* `~h_max` (Integer, default: `127`)
* `~h_min` (Integer, default: `-128`)
* `~s_max` (Integer, default: `255`)
* `~s_min` (Integer, default: `0`)
* `~i_max` (Integer, default: `255`)
* `~i_min` (Integer, default: `0`)

   Color range to filter.

### jsk\_pcl/AddPointIndices
add two different `pcl_msgs/PointIndices` into one indices.

#### Subscribing Topic
* `~input/src1` (`pcl_msgs/PointIndices`)
* `~input/src2` (`pcl_msgs/PointIndices`)

  Input indices

#### Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Output indices

#### Parameters
* `approximate_sync` (Boolean, default: `false`)

  If this parameter is true, `~input/src1` and `~input/src2` are synchronized with
  approximate time policy.

### jsk\_pcl/PolygonToMaskImage
![](images/polygon_to_mask_image.png)

Convert polygon into mask image.

#### Subscribing Topic
* `~input` (`geometry_msgs/PolygonStamped`)

  Input 3-D polygon.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Input camera info to project 3-D polygon.

#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Mask image to fill `~input` polygon. Currently only convex polygon is supported.

### jsk\_pcl/MaskImageFilter
![](images/mask_image_filter.png)

Extract indices of pointcloud which is masked by mask image. The pointcloud is no need to be organized.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input point cloud.
* `~input/mask` (`sensor_msgs/Image`)

  Mask image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameters of the image.

#### Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Indices of the points masked by `~input/mask`.

### jsk\_pcl/ROIToRect
Convert camera info with ROI to `geometry_msgs/PolygonStamped`.

#### Subscribing Topic
* `~input` (`sensor_msgs/CameraInfo`)

  Input camera info with ROI filled.

#### Publishing Topic
* `~output` (`geometry_msgs/PolygonStamped`)

  Output rectangle region.

### jsk\_pcl/ROIToMaskImage
Convert camera info with ROI to mask image.

#### Subscribing Topic
* `~input` (`sensor_msgs/CameraInfo`)

  Input camera info with ROI filled.

#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Output mask image.

### jsk\_pcl/MaskImageToROI
Convert a mask image into camera info with roi.

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Original camera info.

#### Publishing Topic
* `~output` (`sensor_msgs/CameraInfo`)

  Camera info with ROI field filled.

### jsk\_pcl/MaskImageToRect
Convert a mask image into geometry_msgs::PolygonStamped.

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask image.

#### Publishing Topic
* `~output` (`geometry_msgs/PolygonStamped`)

  PolygonStamped message which only contains two points. Minimum point and Maximum point to represent bounding box in image.

### jsk\_pcl/MaskImageToRect
### jsk\_pcl/TorusFInder
![](images/torus_finder.png)

Find a torus out of pointcloud based on RANSAC with 3-D circle model.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud`)

  Input pointcloud. You may need to choose good candidates of pointcloud.

#### Publishing Topic

* `~output` (`jsk_recognition_msgs/Torus`)

  Output of detection.

* `~output/inliers` (`pcl_msgs/PointIndices`)
* `~output/coefficients` (`pcl_msgs/ModelCoefficients`)

  Inliers and coefficients which represents detection result.
* `~output/array` (`jsk_recognition_msgs/TorusArray`)

  Array of torus. It will be used for visualization.

#### Parameters
* `~min_radius` (Double, default: `0.1`)
* `~max_radius` (Double, default: `1.0`)

  Minimum and maximum radius of torus.
* `~min_size` (Integer, default: `10`)

  Minimum number of inliers.
* `~outlier_threshold` (Double, default: `0.01`)

  Outlier threshold used in RANSAC.
* `~max_iterations` (Integer, default: `100`)

  Maximum number of iterations of RANSAC.
### jsk\_pcl/RectToROI
Convert rectangle (`geometry_msgs/Polygon`) into ROI with camera info (`sensor_msgs/CameraInfo`).

We expect it will be used with image_view2.

#### Subscribing Topic
* `~input` (`geometry_msgs/Polygon`)

  Polygon to represent rectangle region of image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Original camera info.

#### Publishing Topic
* `~output` (`sensor_msgs/CameraInfo`)

  camera info with ROI filled by `~input`.

### jsk\_pcl/RectToMaskImage
Convert rectangle (`geometry_msgs/Polygon`) into mask image (`sensor_msgs/Image`)

We expect it will be used with image_view2.

#### Subscribing Topic
* `~input` (`geometry_msgs/Polygon`)

  Polygon to represent rectangle region of image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Original camera info.

#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Mask image.

### jsk\_pcl/AddColorFromImage
![](images/add_color_from_image.png)

Add color to pointcloud (no need to be organized) from image and camera info.

### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud`)

  Input point cloud to add color. No need to be an organized pointcloud.
* `~input/image` (`sensor_msgs/Image`)

  BGR8 color image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameters of `~input/image`

### Publishing Topic
* `~output` (`sensor_msgs/PointCloud`)

  Output colored pointcloud.

### jsk\_pcl/PlaneConcatenator
![](image/plane_concatenator.png)

Concatenate near planes and build new set of planes.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud.
* `~input/indices` (`jsk_recognition_msgs/ClusterPointIndices`)
* `~input/polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~input/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)
  Input planes.

#### Publishing Topics
* `~output/indices` (`jsk_recognition_msgs/ClusterPointIndices`)
* `~output/polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~output/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)
  Concatenated planes. Coefficients parameters are refined by RANSAC.

#### Parameters
* `~connect_angular_threshold` (Double, default: `0.1`)

   Angular threshold to regard two planes as near.
* `~connect_distance_threshold` (Double, default: `0.1`)

   Euclidean distance threshold to regard two planes as near.
* `~ransac_refinement_max_iteration` (Integer, default: `100`)

  The maximum number of iteration of RANSAC refinement.
* `~ransac_refinement_outlier_threshold` (Double, default: `0.1`)

  Outlier threshold of RANSAC refinmenet.
* `~ransac_refinement_eps_angle` (Double, default: `0.1`)

  Eps angle threshold of RANSAC refinment using normal direction of the plane.
### jsk\_pcl/SupervoxelSegmentation
![](images/supervoxel_segmentation.png)

Segment pointcloud based on Supervoxel technique.
see Voxel Cloud Connectivity Segmentation - Supervoxels for Point Clouds (J. Papon et al. CVPR2013).
#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. It should have rgb field.

#### Publishing Topic
* `~output/cloud` (`sensor_msgs/PointCloud2`)

  Output pointcloud downsampled by voxel grid.

* `~output/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

  Clustering result.

#### Parameters
* `~color_importance` (`Double`, default: `0.2`)

  Color importance factor.
* `~spatial_importance` (`Double`, default: `0.4`)

  Spatial importance factor.
* `~normal_importance` (`Double`, default: `1.0`)

  Normal importance factor.
* `~use_transform` (`Boolean`, default: `True`)

  Use single cloud transform
* `~seed_resolution` (`Double`, default: `0.1`)

  Seed resolution of super voxels.
* `~voxel_resolution` (`Double`, default: `0.008`)

  Voxel grid resolution of super voxels.

### jsk\_pcl/IncrementalModelRegistration
#### What Is This
![](images/incremental_model_registration.png)

Build a full-model from sequential captured data.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud. RGB field is required.
* `~input/pose` (`geometry_msgs/PoseStamped`)

  Initial pose to estimate acculate pose of the pointcloud.
* `~input/indices` (`pcl_msgs/PointIndices`)

  Indices to mask object in `~input` pointcloud.

#### Publishing Topic
* `~output/non_registered` (`sensor_msgs/PointCloud2`)

  Pointcloud just concatenated according to `~input/pose`

* `~output/registered` (`sensor_msgs/PointCloud2`)

  Pointcloud refined by ICP.
#### Using Services
* `~icp_service` (`jsk_pcl_ros/ICPAlign`)

  ICP service interface to refine model.

### jsk\_pcl/IntermittentImageAnnotator
#### What Is This
![](images/intermittent_image_annotator.png)
![](images/intermittent_image_annotator_pointcloud_clip.png)

1. Store images when `~shutter` service is called
2. Publish snapshots as one concatenated image
3. Subscribe `~output/screenrectangle` to get ROI.
4. Publish ROI information to `~output` namespace.
5. Publish pointcloud inside of the ROI to `~output/cloud` if `~store_pointcloud` is set

#### Subscribing Topic
* `~input/image` and `~input/camera_info` (`sensor_msgs/Image` and `sensor_msgs/CameraInfo`)

  Input image and camera info.

* `~input/cloud` (`sensor_msgs/PointCloud2`)

  Input pointcloud to be clipped by specified ROI.

* `~output/screenrectangle` (`geometry_msgs/PolygonStamped`)

  ROI. We expect to use [image_view2](https://github.com/jsk-ros-pkg/jsk_common/tree/master/jsk_ros_patch/image_view2).

#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Snapshots as one concatenated image.

* `~output/direction` (`geometry_msgs/PoseStamped`)

  Direction of ROI as `PoseStamped`. z-axis directs the center of ROI.

* `~output/roi` (`jsk_recognition_msgs/PosedCameraInfo`)

  Publish ROI of specified region as `PosedCameraInfo`.

* `~output/cloud` (`sensor_msgs/PointCloud`)

  Pointcloud inside of ROI. pointcloud is stored when `~shutter` service is called and
  its timestamp will be updated according to the latest image.

* `~output/marker` (`visualization_msgs/Marker`)

  Marker to visualize ROI (`~output/roi`).
#### Parameters
* `~fixed_frame_id` (`String`, default: `odom`)

  Fixed frame id to resolve tf.

* `~max_image_buffer` (`Integer`, default: `5`)

  The maximum number of images to store in this nodelet.

* `~store_pointcloud` (`Boolean`, default: `false`)

  Store pointcloud if it's true

* `~keep_organized` (`Boolean`, default: `false`)

  Keep pointcloud organized after clipping by specified ROI.

#### Advertising Service

* `~shutter` (`std_srvs/Empty`)

  Take a snapshot

* `~clear` (`std_srvs/Empty`)

  Clear images stored in the nodelet.

### jsk\_pcl/LINEMODDetector
#### What Is This
![](images/linemod_detector.png)

A nodelet to detect object using LINEMOD.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud.

#### Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Result of detection as pointcloud.

* `~output/mask` (`sensor_msgs/Image`)

  Result of detection as mask image.

* `~output/pose` (`geometry_msgs/PoseStamped`)

  Pose of detected template

* `~output/template` (`sensor_msgs/PointCloud2`)

  Template pointcloud at identity pose.

#### Parameters
* `~template_file` (`String`, default: `template`)

  Template file
* `~gradient_magnitude_threshold` (`Double`, default: `10.0`)

  Gradient maginutude threshold

* `~detection_threshold` (`Double`, default: `0.75`)

  Detection threshold

### jsk\_pcl/LINEMODTrainer
#### What Is This
![](images/linemod_trainer.png)


A nodelet to train LINEMOD data from pointcloud and indices to mask the objects.
This nodelet stores data of pointcloud and if you call `~start_training` service,
it will train the data and dump the templates into lmt file.

#### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  This pointcloud should be able to be converted into `pcl::PointXYZRGBA` data.
* `~input/indices` (`pcl_msgs/PointIndices`)

  Indices to mask object in `~input` pointcloud.

* `~input/info` (`sensor_msgs/CameraInfo`)

  Camera parameter to sample viewpoint.

#### Publishing Topic
* `~output/range_image` (`sensor_msgs/Image`)
* `~output/colored_range_image` (`sensor_msgs/Image`)
* `~output/sample_cloud` (`sensor_msgs/PointCloud2`)

  Image and pointcloud generated by viewpoint sampling.

#### Advertising Servicies
* `~start_training` (`std_srvs/Empty`)

  Start training and dump result into a file.

* `~clear_data` (`std_srvs/Empty`)

  Clear stored data.
#### Parameters
* `~output_file` (`String`, default: `template`)

   A file path to dump trained data.

* `~sample_viewpoint` (`Bool`, default: `True`)

  Generate training data by samplingenerating viewpoint if this parameter is set to true.

* `~sample_viewpoint_angle_step` (`Double`, default: `40.0`)
* `~sample_viewpoint_angle_min` (`Double`, default: `-80.0`)
* `~sample_viewpoint_angle_max` (`Double`, default: `80.0`)
* `~sample_viewpoint_radius_step` (`Double`, default: `0.2`)
* `~sample_viewpoint_radius_min` (`Double`, default: `0.4`)
* `~sample_viewpoint_radius_max` (`Double`, default: `0.8`)

  Viewpoint sampling parameters. Pose of model is sampled by golden ratio spatial technique.

### jsk\_pcl/CaptureStereoSynchronizer
#### What Is This
![](images/capture_stereo_synchronizer.png)

A nodelet to capture training data of stereo cameras. It subscribe several messages with
synchronizing timestamp and republish them into `~output` namespace.

#### Subscribing Topic
* `~input/pose` (`geometry_msgs/PoseStamped`)

  Pose of checkerboard

* `~input/mask` (`sensor_msgs/Image`)

  Mask image of the object

* `~input/mask_indices` (`pcl_msgs/PointIndices`)

  Pointcloud indices of the object

* `~input/left_image` (`sensor_msgs/Image`)

  Left camera image

* `~input/left_camera_info` (`sensor_msgs/CameraInfo`)

  Left camera parameter

* `~input/right_camera_info` (`sensor_msgs/CameraInfo`)

  Right camera parameter

* `~input/disparity` (`stereo_msgs/Disparity`)

  Disparity image of the stereo camear.

#### Publishing Topic
* `~output/pose` (`geometry_msgs/PoseStamped`)
* `~output/mask` (`sensor_msgs/Image`)
* `~output/mask_indices` (`pcl_msgs/PointIndices`)
* `~output/left_image` (`sensor_msgs/Image`)
* `~output/left_camera_info` (`sensor_msgs/CameraInfo`)
* `~output/right_camera_info` (`sensor_msgs/CameraInfo`)
* `~output/disparity` (`stereo_msgs/Disparity`)

  These topics are the same message to the `~input/foo` messages but all of them
  are republished only if input messages are synchronized.

#### Sample
Please check [capture_multisense_training_data.launch](launch/capture_multisense_training_data.launch).

### jsk\_pcl/MaskImageToPointIndices
A nodelet to convert mask image (`sensor_msgs::Image`) to `pcl_msgs/PointIndices` for
organized pointcloud.

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask image.

#### Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Output indices converted from the mask image.

### jsk\_pcl/PointIndicesToMaskImage
#### What Is This
![](images/point_indices_to_mask_image.png)

jsk\_pcl/PointIndicesToMaskImage generates mask image from `pcl_msgs/PointIndices`
of organized pointcloud and original `sensor_msgs/Image`.

#### Subscribing Topic
* `~input` (`pcl_msgs/PointIndices`)

   Indices of the point cloud to mask.

* `~input/image` (`sensor_msgs/Image`)

   In order to know width and height of the original image, jsk\_pcl/PointIndicesToMaskImage requires
   input image.

#### Publishing Topic

* `~output `(`sensor_msg/Image`)


   Mask image to get `~input` indices from the origina limage.

### jsk\_pcl/AttentionClipper
#### What Is This
![](images/attention_clipper.png)

It retrives `sensor_msgs/CameraInfo` and publish `sensor_msgs/CameraInfo` with ROI filled and
retirieves `sensor_msgs/PointCloud2` and publish `pcl_msgs/PointIndices`.

You can specify the pose and size of the interest bounding box and jsk\_pcl/AttentionClipper returns ROI
to see the object.

#### Subscribing Topic
* `~input` (`sensor_msgs/CameraInfo`)

  Original camera info.

* `~input/points` (`sensor_msgs/PointCloud2`)

  Original pointcloud.
* `~input/pose` (`geometry_msgs/PoseStamped`)
* `~input/box` (`jsk_recognition_msgs/BoundingBox`)
  Specify the pose of the bounding box. Timestamp will be ignored and camera info's timestamp will be used. If you use `~input/box`, you can change the size of attention region. There callbacks are only enabled if `~use_multiple_attention` is false.

* `~input/pose_array` (`geometry_msgs/PoseArray`)
* `~input/box_array` (`jsk_recognition_msgs/BoundingBoxArray`)
  It's an array version of `~input/pose` and `~input/box`. There callbacks are only enabled if `~use_multiple_attention` is true.

#### Publishing Topic
* `~output` (`sensor_msgs/CameraInfo`)

  This camera info is same with `~input` except for roi field.

* `~output/mask` (`sensor_msgs/Image`)

  Mask image to mask the regions of specified interest.

* `~output/point_indices` (`pcl_msgs/PointIndices`)

  Indices of `~input/points` which are inside of interest regions.

#### Parameter
* `~use_multiple_attention` (Boolean, default: `False`)

  If you want to enable multiple attentions, please set this variable True.

* `~dimension_x` (Double, default: `0.1`)
* `~dimension_y` (Double, default: `0.1`)
* `~dimension_z` (Double, default: `0.1`)

  Size of bounding box. Available only if `~use_multiple_attention` is false.

* `~frame_id` (String, default: `base_link`)

  Frame id of attention region. Available only if `~use_multiple_attention` is false.

* `~initial_pos` (Array of double, default: `None`):

  Initial pose of interesting region. Available only if `~use_multiple_attention` is false.

* `~initial_rot` (Array of double, default: `None`):

  Initial orientation of interesting region. The value should be represented in
  [roll, pitch, yaw]. Available only if `~use_multiple_attention` is false.

* `~initial_pos_list` (Array of array of double, default: `None`)
* `~initial_rot_list` (Array of array of double, default: `None`)
* `~frame_id_list` (Array of string, default: `None`)
* `~dimensions` (Array of array of double, default: `None`)

  Position, Rotation, frame id and Dimensions of multiple attention regions respectively.
  `~iniital_pos_list` should follow `[[x, y, z], ...]`,
  `~initial_rot_list` should follow `[[rx, ry, rz], ...]` and
  `~dimensions` should follow `[[x, y, z], ...]`.
  Available only if `~use_multiple_attention` is true.
### jsk\_pcl/ROIClipper
#### What Is This
![](images/attention_clipper.png)

![](images/roi_clipper_pointcloud.png)

It retrives `sensor_msgs/Image` and `sensor_msgs/CameraInfo` and publish `sensor_msgs/Image` of ROI.
It is similar to `image_proc/crop_decimate` but you can use `CameraInfo/roi` field to specify ROI.

We expect to use jsk\_pcl/ROIClipper with jsk\_pcl/AttentionClipper to get ROI image.

#### Subscribing Topic
* `~input/image` (`sensor_msgs/Image`)

  Input image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameter and ROI field should be filled.

  These two topic should be synchronized if `~not_sync` is not false.

* `~input/cloud` (`sensor_msgs/PointCloud2`)

  This topic is only enabled if `~not_sync` is true.

#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Image of ROI.

* `~output/point_indices` (`pcl_msgs/PointIndices`)

  The indices of the pointcloud which is inside of the interest 3-D region.

* `~output/cloud` (`sensor_msgs/PointCloud2`)

  PointCloud clipped from `~input/cloud` and `~input/camera_info`.

#### Parameter
* `~not_sync` (Bool, default: `False`)

  If ~not_sync is true, do not need to synchronize camera info and other input topics, and
  pointcloud clipping is enabled.
### jsk\_pcl/NormalDirectionFilter
![NormalDirectionFilter](images/normal_direction_filter.png)

jsk\_pcl/NormalDirectionFilter filters pointcloud based on the direction of the normal.
It can filters pointcloud based on **static** direction and direction based on imu linear_acceleration.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`)

   input normal pointcloud.
* `~input_imu` (`sensor_msgs/Imu`)

   imu message, which is enabled if `~use_imu` parameter is true

#### Publishing Topics
* `~output` (`pcl_msgs/PointIndices`)

   result of filtering as indices. You can use `pcl/ExtractIndices` to get pointcloud of the indices.

#### Parameters
* `~use_imu` (Boolean, default: `False`):

   Enable `~input_imu` topic and set target direction based on imu linear acceleration.
* `~eps_angle` (Double, default: `0.2`):

   Eps angle difference to regard the normal as required direction.
* `~angle_offset` (Double, default: `0.0`):

   Offset parameter to the direction.
* `~direction` (Double Array, required):

   if `~use_imu` is false, the direction should be specified with this parmaeter.

### jsk\_pcl/MultiPlaneExtraction
![MultiPlaneExtraction](images/multi_plane_extraction.png)

Extract the points above the planes between `~min_height` and `~max_height`.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   Input pointcloud.
* `~indices` (`jsk_recognition_msgs/ClusterPointIndices`)
* `~input_polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~input_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

   The input planes. If `~use_indices` parameter is false, `~indices` will not be used.

#### Publishing Topics
* `~output` (`sensor_msgs/PointCloud2`):

   Pointcloud above the planes between `~min_height` and `~max_height`.
* `~output_nonplane_cloud` (`sensor_msgs/PointCloud2`):

   Pointcloud above the planes is not between `~min_height` and `~max_height`.

#### Parameters
* `~min_height` (Double, default: `0.0`)
* `~max_height`(Double, default: `0.5`)

   Minimum and maximum height of 3-D polygonal region to extract points.
* `~max_queue_size` (Integer, default: `100`)

   Queue length for subscribing topics.

* `~use_indices` (Bool, default: `True`)

   Use indices of planar regions to filter if it's set true.
   You can disable this parameter to filter pointcloud which is not the same pointcloud
   to segment planes

* `~magnify` (Double, default: `0.0`)

  Magnify planes by this parameter. The unit is m.
### jsk\_pcl/RegionGrowingMultiplePlaneSegmentation
![jsk_pcl/RegionGrowingMultiplePlaneSegmentation](images/region_growing_multiple_plane_segmentation.png).

jsk\_pcl/RegionGrowingMultiplePlaneSegmentation estimates multiple plenes from pointcloud.


It extracts planes based on [region growing](http://en.wikipedia.org/wiki/Region_growing)
and evaluation function of connectivity if based on the following equation:
![](images/region_growing_multiple_plane_segmentation_eq.gif)

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.
* `~input_normal` (`sensor_msgs/PointCloud2`):

   normal pointcloud of `~input`

#### Publishing Topics
* `~output/inliers` (`jsk_recognition_msgs/ClusterPointIndices`):

   Set of indices of the polygons.
* `~output/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

   Array of coefficients of the polygons.
* `~output/polygons` (`jsk_recognition_msgs/PolygonArray`):

   Polygons

#### Parameters
* `~angular_threshold` (Double, default: `0.04`)

   Angular threshold to connect two points in one cluster. See
* `~distance_threshold` (Double, default: `0.1`)

   Distance threshold to connect two points in one cluster.
* `~max_curvature` (Double, default: `0.1`)

   Before extracting planes, filtering out the points which have higer curvature than this value.
* `~min_size` (Integer, default: `100`)

   The minimum number of the points of each plane.
* `~cluster_tolerance` (Double, default: `0.1`)

   The spatial tolerance for new cluster candidates.
* `~ransac_refine_outlier_distance_threshold` (Double, default: `0.1`)

   Outlier threshold for plane estimation using RANSAC.
* `~ransac_refine_max_iterations` (Integer, default: `100`)

   The maximum number of the iterations for plane estimation using RANSAC.

### jsk\_pcl/ParticleFilterTracking
#### What Is This
![](images/particle_filter_tracking.png)

This nodelet tracks the target pointcloud.

##### Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud

* `~renew_model` (`sensor_msgs/PointCloud2`)

  Reference pointcloud to tracke.

* `~renew_box` (`jsk_recognition_msgs/BoundingBox`)

  Bounding box information to align reference pointcloud model. Only if availabel `~align_box` parameter is true.

##### Publishing Topic
* `~track_result` (`sensor_msgs/PointCloud2`)

  Reference pointcloud which is transformed by tracking result.

* `~tracking_result_pose` (`geometry_msgs/PoseStamped`)

  Tracking result as pose of reference pointcloud.

* `~particle` (`sensor_msgs/PointCloud2`)

  Particles during tracking. Only x, y and z are available.

##### Advertising Servicies
* `~renew_model` (`jsk_pcl_ros/SetPointCloud2`)

  Service interface to set reference pointcloud.

##### Parameters
* `~thread_nr` (Integer, default: `cpu num`)

  The number of thread used in tracking
* `~particle_num` (Integer, default: `~max_particle_num`)

  The number of initial particles
* `~use_normal` (Boolean, default: `false`)

  Use normal information to track or not.
* `~use_hsv` (Boolean, default: `true`)

  If it's true, tracker uses color information in HSV color space to
  evaluate likelihood.
* `~track_target_name` (Boolean, default: `track_result`)

  The name of the target, it is used as frame_id of tf.
* `~octree_resolution` (Double, default: `0.01`)

  Octree resolution to search.
* `~align_box` (Bool, default: `false`)

  If it's true, tracker subscribes `~renew_box` topic and align reference model against the box.
* `~BASE_FRAME_ID` (String, default: `NONE`)

  Coordinate system of the tracker. `NONE` means "same to frame_id of input poiintcloud".
* `~default_initial_mean` (Array of double, default: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`)

  Mean value of initial sampling.
* `~initial_noise_covariance` (Array of double, default: `[0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]`)

  Covariance value of initial sampling.

* `~max_particle_num` (Integer, default: `1000`)

  Maximum numebr of particles

* `~iteration_num` (Integer, defeault: `1`)

  The number of iteration per one frame.
* `~resample_likelihood_thr` (Double, default: `0.0`)

  Threshold of likelihood to resample (re-initialize) all the particles.
* `~delta` (Double, default: `0.09`)

  Delta value for KLD sampling.
* `~epsilon` (Double, default: `0.2`)

  epsilon parameter for KLD sampling.
* `~bin_size_x` (Double, default: `0.01`)
* `~bin_size_y` (Double, default: `0.01`)
* `~bin_size_z` (Double, default: `0.01`)
* `~bin_size_roll` (Double, default: `0.01`)
* `~bin_size_pitch` (Double, default: `0.01`)
* `~bin_size_yaw` (Double, default: `0.01`)

  Size of bin for KLD sampling. Larger value means smaller number of particles.
* `~default_step_covariance_x` (Double, default: 0.00001)
* `~default_step_covariance_y` (Double, default: 0.00001)
* `~default_step_covariance_z` (Double, default: 0.00001)
* `~default_step_covariance_roll` (Double, default: 0.00001)
* `~default_step_covariance_pitch` (Double, default: 0.00001)
* `~default_step_covariance_yaw` (Double, default: 0.00001)

  Covariance value of noise in resampling phase.

* `~reversed` (Boolean, default: `false`)

  Reverse relationship between reference and input. If this parameter is true,
  tracker transforms input pointcloud instead of reference pointcloud.
  It is useful when input pointcloud is smaller than reference pointcloud.

  If this parameter is true, KLDSampling is disabled.
* `~not_use_reference_centroid` (Boolean, default: `false`)

  If this parameter is true, tracker des not use centroid of reference pointcloud as the origin of reference pointcloud.

* `~not_publish_tf` (Boolean, default: `false`)

  If this parameter is true, do not publish tf frame.

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
![](images/tilt_laser_listener.png)

Listen to the joint_states of tilt/spindle laser and publish time range to scane full 3-D space.
You can choose several types of tilt/spindle lasers such as tilt-laser of PR2, infinite spindle laser of multisense.

#### Subscribing Topics
* `~input`(`sensor_msgs/JointState`):

   Joint angles of laser actuator.

#### Publishing Topics
* `~output` (`jsk_recognition_msgs/TimeRange`):

   Time range to scan 3-D space.
* `~output_cloud` (`sensor_msgs/PointCloud2`):

   Assembled pointcloud according to time range
   of `~output`. this require `~assemble_scans2`
   service of [laser_assembler](http://wiki.ros.org/laser_assembler).

#### Using Services
* `~assemble_scans2` (`laser_assembler/AssembleScans2`):

   A service to build 3-D pointcloud from scan pointcloud.
   It should be remapped to `assemble_scans2` service of
   [laser_assembler](http://wiki.ros.org/laser_assembler).

#### Advertising Service
* `~clear_cache` (`std_srvs/Empty`)

   Clear cache and restart collecting data.

#### Parameters
* `~use_laser_assembler` (Boolean, default: `False`):

   Enable `~output_cloud` and `~assemble_scans2`.
* `~joint_name` (String, **required**):

   Joint name of actuator to rotate laser.
* `~laser_type` (String, default: `tilt_half_down`):

   type of rotating laser. You can choose one of the types:
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
* `~overwrap_angle` (Double, default: `0.0`)

   overwrap angle offset when detecting time range.
   Only available in `infinite_spindle` and `infinite_spindle_half`.

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

The result of clustering is published as `jsk_recognition_msgs/ClusterPointIndices`.

If the number of the cluster is not changed across different frames, `EuclideanClustering`
tries to track the segment.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.

#### Publishing Topics
* `~output` (`jsk_recognition_msgs/ClusterPointIndices`):

   Result of clustering.
* `~cluster_num` (`jsk_recognition_msgs/Int32Stamped`):

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
Decompose `jsk_recognition_msgs/ClusterPointIndices` into array of topics of `sensor_msgs/PointCloud` like `~output00`, `~output01` and so on.
It also publishes tf of centroids of each cluster and oriented bounding box of them. The direction of the bounding box are aligned on to the nearest planes if available.

#### Subscribing topics
* `~input` (`sensor_msgs/PointCloud2`):

   Input pointcloud.
* `~target` (`jsk_recognition_msgs/ClusterPointIndices`):

   Input set of indices to represent clusters.
* `~align_planes` (`jsk_recognition_msgs/PolygonArray`):
* `~align_planes_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`):

   The planes for bounding box to be aligned on.

#### Publishing topics
* `~output%02d` (`sensor_msgs/PointCloud2`):

   Series of topics for each pointcloud cluster.
* `~debug_output` (`sensor_msgs/PointCloud2`):

   Concatenate all the clusters into one pointcloud and colorize each cluster to see the result of segmentation.
* `~boxes` (`jsk_recognition_msgs/BoundingBoxArray`):

   Array of oriented bounding box for each segmented cluster.

#### Parameters
* `~publish_tf` (Boolean, default: `True`):

   Toggle tf publishing.
* `~publish_clouds` (Boolean, default: `True`):

   Toggle `~output%02d` topics.
* `~align_boxes` (Boolean, default: `False`):

   If this parameter is set to `True`, `~align_planes` and
   `~align_planes_coefficients` are enabled.
* `~use_pca` (Boolean, default: `False`):

   Run PCA algorithm on each cluster to estimate x and y direction.

### jsk\_pcl/ClusterPointIndicesDecomposerZAxis
#### What Is This
This nodelet is almost same to jsk\_pcl/ClusterPointIndicesDecomposer, however it always sort clusters in z direction.

### jsk\_pcl/CentroidPublisher
#### What Is This

This nodelet will subscribe the sensor\_msgs::PointCloud2, calculate its centroid  and boardcast the tf whose parent is cloud headers frame\_id and whose child is the new centroid frame_id.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   input pointcloud.
#### Publishing Topics
* `/tf`:

   Publish tf of the centroid of the input pointcloud.

#### Parameters
* `~frame` (String, required):

   frame_id of centroid tf

#### Sample
Plug the depth sensor which can be launched by openni.launch and run the below command.

```
roslaunch jsk_pcl_ros centroid_publisher.launch
```

### jsk\_pcl/OrganizedMultiPlaneSegmentation
#### What Is This
![images/organized_multi_plane_segmentation.png](images/organized_multi_plane_segmentation.png)

This nodelet segments multiple planes from **organized** pointcloud.
It estimates planes based on [connected-component analysis](http://en.wikipedia.org/wiki/Connected-component_labeling)
using `pcl::OrganizedMultiPlaneSegmentation`.

![overview](images/graph/organized_multi_plane_segmentation_overview.png) shows the overview of the pipeline.

1. Estimate normal using integral image.
2. Conduct connected component analysis to estimate planar regions.
3. Connect neighbor planes if the normal directions and the borders of the planes are near enough.
4. Refine plane coefficients of connected planes based on RANSAC. If the areas of the planes after refinement are too small, they will be removed.

These process is implemented in one nodelet in order not to convert pointcloud between
PCL and ROS.

#### Subscribing Topics
* `~input` (`sensor_msgs/PointCloud2`):

   Input pointcloud. This should be **organized** pointcloud.

#### Publishing topisc
* `~output` (`jsk_recognition_msgs/ClusterPointIndices`):
* `~output_polygon` (`jsk_recognition_msgs/PolygonArray`):
* `~output_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

   The inliers, coefficients and convex polygons of the connected polygons.
* `~output_nonconnected` (`jsk_recognition_msgs/ClusterPointIndices`):
* `~output_nonconnected_polygon` (`jsk_recognition_msgs/PolygonArray`):
* `~output_nonconnected_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

   The inliers, coefficients and polygons of the polygons of connected components analysis.
* `~output_refined` (`jsk_recognition_msgs/ClusterPointIndices`):
* `~output_refined_polygon` (`jsk_recognition_msgs/PolygonArray`):
* `~output_refined_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

   The inliers, coefficients and convex polygons of the refined polygons.
* `~output_normal` (`sensor_msgs/PointCloud2`):

   The pointcloud of normal of `~input` pointcloud.

#### Parameters
* `~estimate_normal` (Boolean, default: `True`):

   Estimate normal if it is set to `True`
* `~publish_normal` (Boolean, default: `False`):

   Publish the result of normal to `~output_normal`
* `~max_depth_change_factor` (Double, default: `0.02`):

   The depth change threshold for computing object borders in normal estimation.
* `~normal_smoothing_size` (Double, default: `20.0`):

   the size of the area used to smooth normals
   (depth dependent if `~depth_dependent_smoothing` is true)
* `~depth_dependent_smoothing` (Boolean, default: `False`)

   Smooth normal depending on depth
* `~estimation_method` (Integer, default: `1`)

   Estimation method of normal. You can choose one of `AVERAGE_3D_GRADIENT(0)`, `COVARIANCE_MATRIX(1)` and `AVERAGE_DEPTH_CHANGE(2)`.
* `~border_policy_ignore` (Boolean, default: `True`)

   Ignore border if this is `True`
* `~min_size` (Integer, default: `2000`)

   Minimum number of the points on a planar region during connected component analysis.
   We recommend smaller size for this parameter in order to get stable result.
* `~angular_threshold` (Double, default: `0.05`)
* `~distance_threshold` (Double, default: `0.01`)

   Distance and angular threshold in connected component analysis.
* `~max_curvature` (Double, default: `0.001`)

   The maximum curvature allowed for a planar region
* `~connect_plane_angle_threshold` (Double, default: `0.2`)
* `~connect_distance_threshold` (Double, default: `0.01`)

   These parameters affect near plane connection. OrganizedMultiPlaneSegmentation connects
   planes which have near normal direction and whose boundaries are near enough.
* `~ransac_refine_coefficients` (Boolean, default: `True`)

   Conduct RANSAC refinment for each plane if it is true.
* `~ransac_refine_outlier_distance_threshold` (Double, default: `0.1`)

   Outlier threshold of RANSAC refinment for each plane.
* `~min_refined_area_threshold` (Double, default: `0.04`)
* `~max_refined_area_threshold` (Double, default: `10000`)

   Minimum and maximum area threshold for each convex polygon.

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
![](images/hinted_plane_detector.png)
Estimate plane parameter from small 'hint' pointcloud and grow it to detect larger plane.

Algorithm is:

1. Detect hint plane from small hint pointcloud using RANSAC
2. Filter `~input` pointcloud based on distance and normal direction with hint plane.
3. Detect plane from the pointcloud using RANSAC
4. Segment clusters out of the inliers of the detected plane based on euclidean metrics
5. Apply density filter
6. Extract points from the nearest segmented clusters to the centroid of hint plane
7. Compute convex hull of the extracted points

#### Subscribing Topic

* `~input` (`sensor_msgs/PointCloud`)

  Input pointcloud. It is required to have normal and xyz fields.

* `~input/hint/cloud` (`sensor_msgs/PointCloud`)

  Hint pointcloud to estimate plane parameter and only xyz fieleds are required.

#### Publishing Topic

* `~output/polygon` (`geometry_msgs/PolygonStamped`)
* `~output/polygon_array` (`jsk_recognition_msgs/PolygonArray`)
* `~output/inliers` (`pcl_msgs/PointIndices`)
* `~output/coefficients` (`pcl_msgs/ModelCoefficients`)

  Result of detection.

* `~output/hint/polygon` (`geometry_msgs/PolygonStamped`)
* `~output/hint/polygon_array` (`jsk_recognition_msgs/PolygonArray`)
* `~output/hint/inliers` (`pcl_msgs/PointIndices`)
* `~output/hint/coefficients` (`pcl_msgs/ModelCoefficients`)

  Result of detection of hint pointcloud.
* `~output/polygon_before_filtering` (`geometry_msgs/PolygonStamped`)
* `~output/polygon_array_before_filtering` (`jsk_recognition_msgs/PolygonArray`)

  Result of detection before euclidean filtering.
* `~output/hint_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/plane_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/density_filtered_indices` (`pcl_msgs/PointIndices`)
* `~output/euclidean_filtered_indices` (`pcl_msgs/PointIndices`)

  Candidate point indices filtered by each filtering phase.

#### Parameters
* `~hint_outlier_threshold`

  Outlier threshold to detect hint plane using RANSAC
* `~hint_max_iteration`

  Maximum iteration number to detect hint plane using RANSAC
* `~hint_min_size`

  Minimum number of inliers in hint plane
* `~outlier_threashold`

  Outlier threshold to detect larger plane using RANSAC
* `~max_iteration`

  Maximum iteration number to detect larger plane using RANSAC
* `~min_size`

  Minimum number of inliers in larger plane
* `~eps_angle`

  EPS angle to detect larger plane and normal direction is computed from hint plane.
* `~normal_filter_eps_angle`

  EPS angle to filter candidate points before detecting larger plane.
  Normal direction is computed from hint plane.
* `~euclidean_clustering_filter_tolerance`

  Tolerance distance in euclidean clustering to filter far points.
* `~euclidean_clustering_filter_min_size`

  Minimum cluster size in euclidean clustering to filter far points.

* `~density_radius` (Double, default: `0.1`)
* `~density_num` (Integer, default: `10`)

  These parameters are used in density filtering. The only points which have `~density_num` neighbors within
  `~density_radius` distance are passed.

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
