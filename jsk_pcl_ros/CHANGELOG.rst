^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pcl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.17 (2015-08-21)
-------------------

0.2.16 (2015-08-19)
-------------------
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Compute likelihood based on plane-detection-sensor
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Remove unused parameters from class member
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] More correct border condition about occlusion
* Remove files which added by mistake
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Support sensor_frame via  ~sensor_frame parameter
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Separate likelihood computation parameters from particlefilter parameter to cleanup dynamic_reconfigure parameters
* [jsk_pcl_ros] Add InteractiveCuboidLikelihood to confirm behavior of likelihood function of PlaneSupportedCuboidEstimator by interactive server
* Contributors: Ryohei Ueda

0.2.15 (2015-08-18)
-------------------
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Add ~min_inliers and
  ~outlier_distance parameter
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Take occlusion into account
* [jsk_pcl_ros/PlaneSupportedCuboidEstimator] Add ~use_range_likelihood to
  toggle use likelihood based on geometric constraint
* Merge pull request `#1054 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1054>`_ from garaemon/plane-supported-cuboid-estimator
  [jsk_pcl_ros/PlaneSupportedCuboidestimator] Add new nodelet to estimate cuboid on plane based on bayesian recursive estimation
* [jsk_pcl_ros/PlaneSupportedCuboidestimator] Add new nodelet to estimate
  cuboid on plane based on bayesian recursive estimation, especially
  particle filter is used.
* [jsk_pcl_ros] Add simple code and script to bench RANSAC based plane estimation
* [jsk_pcl_ros/AttentionClipper] Fix compilation warning:
  1. fixing name confliction of iteration index
  2. Use std::runtime_error to catch exception
* [jsk_pcl_ros] Super simple script to plot gaussian. It is useful to
  determin several parameters based on normal distribution
* [jsk_pcl_ros] fix flip option
* Contributors: Ryohei Ueda, Hitoshi Kamada

0.2.14 (2015-08-13)
-------------------
* [jsk_pcl_ros/PoseWithCovarianceStampedtoGussianPointCloud] Add new
  normalize method: normalize_area and normalize_height
* [jsk_pcl_ros/PoseWithCovarianceStampedtoGussianPointCloud] Fix to apply sqrt
* [jsk_pcl_ros/PoseWithCovarianceStampedtoGussianPointCloud] Add offset to z-height
* [jsk_pcl_ros] Update image on readme about PoseWithCovarianceStampedToGaussianCloud
* machine tag should defined in somewhere else, not here
* [jsk_pcl_ros] Add new nodelet to convert geometry_msgs/PoseWithCovarianceStamped to PointCloud with
  gaussian distribution
* [jsk_pcl_ros] fix typo in multi_resolution_organized_pointcloud.launch
* [jsk_pcl_ros/multi_resolution_organized_pointcloud.launch] Add throttled images
* change frame for renew pose
* add options for use self_filter
* add srv to renew pose
* change remapping for stereo resizer
* [jsk_pcl_ros] Use fmod to detect jamp where tilt joint angle continues to inclease, such as gazebo simulation environment
* [jsk_pcl_ros/PolygonPointsSampler] Publich cloud of pcl::PointXYZ
* [jsk_pcl_ros/HeightmapTimeAccumulation] Fix to return true in reset callback
* [jsk_pcl_ros/HeightmapTimeAccumulation] Add ~reset service to clear cache
* [jsk_pcl_ros/HeightmapTimeAccumulation] Supress message
* [jsk_pcl_ros] Rewrite multi_resolution_organized_pointcloud.launch with jsk_topic_tools/standalone_complexed_nodelet
* [jsk_pcl_ros/HeightmapMorphologicalFiltering] Add config topic to simplify chain heightmap pileline
* [jsk_pcl_ros] Add config topic to chain heightmap configuration
* [jsk_pcl_ros/HeightmapToPointCloud] Fix x-y coordinate value to locate point
  at the center of pixels
* [jsk_pcl_ros] HeightmapTimeAccumulation nodelet to complete heightmap in time series
* [jsk_pcl_ros] Update image of HeightmapMorphologicalFilitering to real
  sensor data
* [jsk_pcl_ros] Use boost::accumulators to compute mean and variance in HeightmapMorphologicalFiltering
* [jsk_pcl_ros/HeightmapMorphologicalFiltering] Update sample image
* [jsk_pcl_ros] Add HeightmapMorphologicalFiltering nodelet
* Merge remote-tracking branch 'origin/master' into publish_cloud_with_pose
* add node for read pcd with pose
* [jsk_pcl_ros] Add HeightmapToPointCloud nodelet to convert heightmap to pointcloud
* [jsk_perception] Add nodelet ColorizeFloatImage to colorize generic float image
* [jsk_pcl_ros] Add HeightmapConverter to convert pointcloud to heightmap
* [jsk_pcl_ros] Add ColorizeHeight2DMapping and move
  ColorizeDistanceFromPlane to libjsk_pcl_util.so
* [jsk_pcl_ros/TiltLaserListener] Add max_queue_size
* [jsk_pcl_ros] add OrganizedNeighbor search method in ParticleFilterTracking
* [jsk_pcl_ros/TfTransformCloud] Use tf::MessageFilter
* [jsk_pcl_ros/stereo_reconstruction.launch] Fix several remappings
* [jsk_pcl_ros] Update stereo_reconstruction.launch for the latest jsk_topic_tools
* move model with pose and take color in condition
* [jsk_pcl_ros] set tracking model with marker in particle_filter_tracker
* [jsk_pcl_ros/TiltLaserListener] Add ~clear_assembled_scans parameter not to
  publish same scans twice
* [jsk_pcl_ros/ICPRegistration] Add parameters for RANSAC
* [jsk_pcl_ros/TiltLaserListener] Do not publish empty pointcloud if buffer is empty
* [jsk_pcl_ros] add pointcloud_to_stl nodelet
* [jsk_pcl_ros] Fix optimization flag
* [jsk_pcl_ros/EnvironmentPlaneModeling] Fix to make it sure to close the
  loop of convex hull
* [jsk_pcl_ros/EnvironmentPlaneModeling] Fix polygon orientation when
  magnify it
* [jsk_pcl_ros] Add diagnostics information to NormalDirectionFilter,
  NormalEstimationOMP and RegionGrowingMultiplePlaneSegmentation
* [jsk_pcl_ros/TfTransformCloud] Add diagnostic information
* [jsk_pcl_ros/NormalFlip] Fix direction of normal flip
* [jsk_pcl_ros/TiltLaserListener] Add diagnostic information
* change default value of max_distance
* fix particle filter tracker
* edit to only remove -std option
* fix c++ version mismatch problem with boost
* [jsk_pcl_ros/handle_estimator.l] change jsk_pcl_ros msgs to jsk_recognition_msgs
* [jsk_pcl_ros] Downsample registered pointcloud for visualization in pointcloud_localizaiton.launch
* [jsk_pcl_ros/PointCloudLocalization] poke vital_checker for diagnostics
* [jsk_pcl_ros] Add launch file to run pointcloud_localization
* [jsk_pcl_ros] Remove model_file argument
* [jsk_pcl_ros] Fix missing display_machine arg
* [jsk_pcl_ros] Do not link gpu libraries if cmake fails to detect PCL_GPU modules
* [jsk_pcl_ros/Kinfu] Publish transformation from map to odom
* [jsk_pcl_ros] Add Kinfu nodelet
* [jsk_pcl_ros/TiltLaserListener] Add ~not_use_laser_assembler_service
  parameter not to use laser_assembler service API but assemble scan
  pointcloud locally
* Contributors: JSK Lab Member, Kei Okada, Kentaro Wada, Ryohei Ueda, Yu Ohara, Yuto Inagaki, Iori Kumagai, Wesley Chan

0.2.13 (2015-06-11)
-------------------
* [jsk_pcl_ros/SnapIt] Reset cached polygons when unsubscribe() is called
* [jsk_pcl_ros] Do not die even if failed to call laser assemble in TiltLaserListener
* [jsk_pcl_ros] Do not close convex polygon when building grid plane
* [jsk_pcl_ros] Add debug message about grid plane construction in EnvironmentPlaneModeling
* [jsk_pcl_ros] Do not publish projected polygon if it failed to estimate 3d point in ScreenPoint
* [jsk_pcl_ros] Support ~always_subscribe in mask_image_filter
* [jsk_pcl_ros] Add ~sensor_frame to MultiPlaneExtraction
* [jsk_pcl_ros] Add waitForTransform to snapit tf resolvance
* [jsk_pcl_ros/RegionGrowingMultiplePlaneSegmentation] Fix computation of
  normal to decide order of vertices by comparing normals from vertices and coefficients
* [jsk_pcl_ros] Untabify attention clipper
* [jsk_pcl_ros/MultiPlaneExtraction] Support negative value for
  magnification of plane
* [jsk_pcl_ros/octree_change_detector] add MACHINE tag to octree_change_detector.launch
* [jsk_pcl_ros] Add ~strict_tf parameter to NormalFlipToFrame to ignore
  timestamp correctness
* add topics for other recognition nodes
* [jsk_pcl_ros] Add NormalEstimationOMP like pcl_ros but it can handle
  timestamp correctly
* [jsk_pcl_ros/EnvironemntPlaneModeling] Add normal direction threshold
* [jsk_pcl_ros/TfTransformPointCloud] Ignore all error in tf conversion
* [jsk_pcl_ros/HintedPlaneDetector] Supress warning messages about pointcloud fields
* [jsk_pcl_ros]add exceptions around tf
* [jsk_pcl_ros] Check if hint convex is valid in HintedPlaneDetector
* [jsk_pcl_ros] Do not publish results if it failes to compute PCA in
  ClusterPointIndicesDecomposer
* [jsk_pcl_ros] Longer TF cache time for TreansformListener which created
  via TFListenerSingleton
* [jsk_pcl_ros/TiltLaserListener] Do not unsubscribe input topics if no
  needed, change it to always subscribe input joint states
* [jsk_pcl_ros] Add new nodelet: NormalFlipToFrame to align direction of
  normal to specified frame_id
* [jsk_pcl_ros] Use jsk_topic_tools/log_utils.h
* [jsk_pcl_ros] Add ~queue_size parameter to NormalDirectionFilter
* [jsk_pcl_ros] Add class and method name to tf error
* [jsk_pcl_ros] Cache result of triangle decomposition
* Contributors: Ryohei Ueda, Yu Ohara, Yuki Furuta

0.2.12 (2015-05-04)
-------------------
* [jsk_pcl_ros] fix attention clipper non nan part
* [jsk_pcl_ros] Add getRadius method to Cylinder
* [jsk_pcl_ros] Remove nan indices from AttentionClipper
* [jsk_pcl_ros] add prefixes params to publish each indices in AttentionClipper
* [jsk_pcl_ros] Set pcl verbosity level to ERROR in multi_plane_extraction
* [jsk_pcl_ros] Relay organized point cloud to "points" topic in stereo_reconstruction.launch
* [jsk_pcl_ros] Ignore tf timestamp in TfTransformPointCloud if ~use_latest_tf is set
* [jsk_pcl_ros] Add stereo_reconstruction.launch to reconstruct stereo
  pointcloud from color images and depth image
* [jsk_pcl_ros] Relay compressed images too in multi_resolution_organized_pointcloud.launch
* [jsk_pcl_ros/mask_image_to_depth_considered_mask_image.cpp] add pcl::removeNaNFromPointCloud
* [jsk_pcl_ros] Resize images in addition to pointcloud
* change input image_points topic to /image_points_color
* [jsk_pcl_ros]change icp result when none reference
* [jsk_pcl_ros] remove nan point before icp kdtree search
* chnage ros-param
* change from linear to non-linear
* modify extract_only_directed_region_of_close_mask_image.launch
* add apply mask image publisher in mask_image_to_depth_considered_mask_image.cpp
* change default parameter of extract num
* rename to NODELET info and short fix
* [jsk_pcl_ros] modify extract_only_directed_region_of_close_mask_image.launch
* [jsk_pcl_ros] resize_points_publisher_nodelet resize rate feedback
* [jsk_pcl_ros] mask_image_to_depth_considered_mask_image_nodelet resize rate feedback
* change default parameter
* rosparam to dynamic-reconfigure
* check if current point is in directed region
* change ROS_ERROR message
* [jsk_pcl_ros] remove duplicate declaration of dependencies
* enable selection of config direction method
* ROS_INFO to ROS_ERROR
* modify README and add image
* [jsk_pcl_ros] add in_the_order_of_depth config
* [jsk_pcl_ros] Add fisheye sphere pub
* Changes to the syntax
* Changes to syntax
* Changes and modification of syntax
* Changes as to the files
* [jsk_pcl_ros] Use rectangle mode for image_view2 in extract_only_directed_region_of_close_mask_image.launch
* add extract_only_directed_region_of_close_mask_image.launch
* [jsk_pcl_ros] extract only directed region of mask image
* changed config name and README
* add dynamic reconfigure config
* [jsk_pcl_ros] Add parameter to skip publishing assembled cloud
* mask image to mask image which is at close range
* Added a launch file for rtabmap mapping with multisense.
* [jsk_pcl_ros] remove unneeded ROS_INFO line
* Contributors: JSK Lab Member, Kamada Hitoshi, Kentaro Wada, Ryohei Ueda, Yohei Kakiuchi, Yoshimaru Tanaka, Yu Ohara, Yuto Inagaki, iKrishneel

0.2.11 (2015-04-13)
-------------------
* [jsk_pcl_ros] Add argument to specify manager name to multi_resolution_pointcloud.launch
* [jsk_pcl_ros] Add several methods and add voxel grid filter to estimate torus
* [jsk_pcl_ros] Keep exact timestamp in AddPointIndices
* Contributors: Ryohei Ueda

0.2.10 (2015-04-09)
-------------------
* [jsk_pcl_ros] generalize namespace of launch value
* [jsk_pcl_ros] Add option to flip z axis direction
* [jsk_pcl_ros] Add geometry_msgs/PolygonStamped input for TorusFinder
* [jsk_pcl_ros] Use simple ros::Subscriber for ResizePointsPublisher
* [jsk_pcl_ros] remove bags in launch
* [jsk_pcl_ros] Supress debug message of AttentionClipper
* [jsk_pcl_ros] change tf fixed frame of config file
* [jsk_pcl_ros] Better caching to handle different frame_id well in attention_clipper
* [jsk_pcl_ros] Resolve tf only once in attention clipper
* [jsk_pcl_ros] Fix projection bug around ConvexPolygon::projectOnPlane
* [jsk_pcl_ros] Fix typo in EnvironmentPlaneModeling
* Contributors: Ryohei Ueda, Yu Ohara


0.2.9 (2015-03-29)
------------------
* 0.2.8
* Update Changelog
* [jsk_pcl_ros] Publish point indices which do not belong to any polygons
  in EnvironmentPlaneModeling
* [jsk_pcl_ros] Erode grid maps as c-space padding in EnvironmentPlaneModeling
* [jsk_pcl_ros] Latch output topic of EnvironmentPlaneModeling
* [jsk_pcl_ros] Check orientation of plane in GridPlane::fromROSMsg
* Contributors: Ryohei Ueda

0.2.8 (2015-03-29)
------------------
* [jsk_pcl_ros] Publish point indices which do not belong to any polygons
  in EnvironmentPlaneModeling
* [jsk_pcl_ros] Erode grid maps as c-space padding in EnvironmentPlaneModeling
* [jsk_pcl_ros] Latch output topic of EnvironmentPlaneModeling
* [jsk_pcl_ros] Check orientation of plane in GridPlane::fromROSMsg
* Contributors: Ryohei Ueda

0.2.7 (2015-03-26)
------------------
* [jsk_pcl_ros] Longer queue size for NormalDirectionFilter
* [jsk_pcl_ros] Implement GridPlane::fromROSMsg method
* Contributors: Ryohei Ueda

0.2.6 (2015-03-25)
------------------
* [jsk_pcl_ros] Publish point with RGB from PolygonPointsSampler
* [jsk_pcl_ros] Set CorrespondenceEstimationOrganizedProjection correctly
* [jsk_pcl_ros] Support ~negative parameter to publish point indices which
  does not inside of attention region
* [jsk_pcl_ros] Support ~use_async in MultiPlaneExtraction
* [jsk_pcl_ros] Clip duplicated pointcloud in PointCloudLocalization
* [jsk_pcl_ros] Add ~use_normal to PointCloudLocalization
* [jsk_pcl_ros] Wait for tf transformation before tansforming pointcloud
* [jsk_pcl_ros] Complete footprint region to the nearest convex polygon in EnvironmentPlaneModeling
* [jsk_pcl_ros] Add PolygonFlipper and fix orientation of convex among
  several nodelets. Force to look upwards in EnvironmentPlaneModeling
* [jsk_pcl_ros] New topic interface to snap pose stamped onto grid map in EnvironmentPlaneModeling
* [jsk_pcl_ros] Do not depends geo_util.h on pcl_conversion_util.h in
  order not to break downstream
* [jsk_pcl_ros] Fix completion of footprint in looking up corresponding
  gridmap in EnvironmentPlaneModeling
* [jsk_pcl_ros] Fill occluded footprint region by bounding box in EnvironmentPlaneModeling
* [jsk_pcl_ros] Add new nodelet to magnify PolygonArray
* [jsk_pcl_ros] Add new sampler to sample pointcloud on polygon with fixed grid
* [jsk_pcl_ros] Add perpendicular distance threshold to PlaneConcatenator
* [jsk_pcl_ros] Add morphological filtering to grid planes
* [jsk_pcl_ros] Add ~input/full_cloud and fix input pointcloud of
  ExtractPolygonalPrismData to close loop of convex hull boundary
* Contributors: Ryohei Ueda

0.2.5 (2015-03-17)
------------------
* [jsk_pcl_ros] Optimize GridPlane::fillCellsFromPointCloud by using
  pcl::ExtractPolygonalPrismData and now it's much much faster than before
* [jsk_pcl_ros] Use pair of index to represent cells of grid
* [jsk_pcl_ros] Refactor EnvironmentPlaneModeling
* check target cloud data ifnot invalid
* add passthrough_image sample launch
* add organized_pc_to_point_indics
* [jsk_pcl_ros] Smaller duration to wait for tf in pointcloud localization
* add approx sync mode to point indices to mask image
* [jsk_pcl_ros]fix miss-name in README
* [jsk_pcl_ros]change ensync timing for plane
* Contributors: Ryohei Ueda, JSK Lab Member, Yu Ohara, Yuto Inagaki

0.2.4 (2015-03-08)
------------------
* [jsk_pcl_ros] Fix coding style of PointcloudScreenpoint
* [jsk_pcl_ros] add ~update_offset service to update localizatoin
  transformation manually
* [jsk_pcl_ros] Add ~use_normal parameter to TorusFinder
* [jsk_pcl_ros] Add hint axis parameter for TorusFinder
  [jsk_pcl_ros] Publish PoseStamped from TorusFinder
* [jsk_pcl_ros] Add service interface to snap footstep to planes in SnapIt
* [jsk_pcl_ros] Publish PoseStamped from TorusFinder
* [jsk_pcl_ros] Add image to PointCloudLocalization document
* [jsk_pcl_ros] Wait tranfrosmtion of tf when clipping pointcloud and
  fix to use y and z dimension of bounding box in AttentionClipper
* [jsk_pcl_ros] Publish PointIndices from ROIClipper to satisfy ROI region
* [jsk_pcl_ros] Fix PointCloudLocalization to work
* [jsk_pcl_ros] Add voxel grid downsampling to keep pointcloud resolution
  constant
* [jsk_pcl_ros] Add PointCloudLocalization for simple SLAM
* [jsk_pcl_ros] Support geometry_msgs/PolygonStamped in SnapIt
* [jsk_pcl_ros] Support polygon input in PointcloudScreenPoint
* [jsk_pcl_ros] Add GeometricConsistencyGrouping nodele
* [jsk_pcl_ros] Add UniformSampling
* [jsk_pcl_ros] Fix FeatureRegistration
* [jsk_pcl_ros] Add FeatureRegistration to register pointclouds using 3D feature
* [jsk_pcl_ros] Add PlanarPointCloudSimulator
* [jsk_pcl_ros] Do not apply PCA for small pointclouds
* Merge pull request `#737 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/737>`_ from garaemon/spherical-cloud-simulator
  [jsk_pcl_ros] Add SphericalPointCloudSimulator nodelet to simulate spindle laser scanner
* [jsk_pcl_ros] Add SphericalPointCloudSimulator nodelet to simulate
  pindle laser scanner
* [jsk_pcl_ros] Add ~use_async parameter to NormalConcatenater
* [jsk_pcl_ros] Fix direction of y-axis of bounding box to direct toward z-axis of pointcloud
* [jsk_pcl_ros] Support normal in ICPRegistration nodelet
* add simple_edge_detector_and_tracker.launch
* [jsk_pcl_ros] add PCL_INCLUDE_DIRS to suppress error of compiling organized_edge_detector
* [jsk_pcl_ros] repair include filed of organized_edge_detector
* [jsk_pcl_ros] Use Eigen::Quaternionf::setFromTwoVectors to align box on plane
* change reversed imu plane direction
* Merge pull request `#728 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/728>`_ from YuOhara/add_hinted_handle_estimator
  Add hinted handle estimator
* add comments
* add_debug_visualizer
* add hinted_handle_estimator
* fix missing include def
* [jsk_pcl_ros] Optimized HintedStickFinder
  1. Use input pointcloud with normal not to run normal estimation in
  HintedStickFinder
  2. Add ~not_synchronize parameter to keep processing without more hint
* [jsk_pcl_ros] Move documentation about
  pointcloud_screenpoint_sample.launch from index.rst to README.md.
  And deprecate sphinx documentation.
* [jsk_pcl_ros] Wait for next new image in shutter callback in IntermittentImageAnnotator
* [jsk_pcl_ros] Deprecate several nodelets
* Merge pull request `#717 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/717>`_ from YuOhara/remove_bags_in_libname
  remove bag in libname
* Merge pull request `#711 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/711>`_ from YuOhara/add_mask_image_indices_concatator
  Add mask image indices concatator
* reversed plane direction
* [jsk_pcl_ros] Return true in clear callback of IntermittentImageAnnotator
* add imu_orientated_plane_rejector
* remove bag in libname
* renamed file name
* rename mask_image_cluster_indices_concatenator to mask_image_cluster_filter
* Merge remote-tracking branch 'origin/master' into add_mask_image_indices_concatator
* [jsk_pcl_ros] Compile without optimization on travis
* [jsk_pcl_ros] Add launch file for torus finder
* [jsk_pcl_ros] Separate moveit filter into libjsk_pcl_ros_moveit
* add topic to sync timestamp
* changed sample_launch for concat indices
* add indices concatenator_node with mask
* renamed node
* add imu_orientated plane detector and launch for icp-use
* [jsk_pcl_ros] changed miss params and comment in data_names out of git
* [jsk_pcl_ros] Add debug printing for tiem stamp confusing problem of resize_point_cloud
* [jsk_pcl_ros] Fix advertise type for template pointcloud:
  geometry_msgs/PoseStamped -> sensor_msgs/PointCloud2
* [jsk_pcl_ros] Fix torus direciton to orient to sensor origin
* [jsk_pcl_ros] Fix detected stick direction always directs to -y upper
* [jsk_pcl_ros] Add PointCloudToClusterPointIndices nodelet
* [jsk_pcl_ros] Publish PointXYZRGBNormal pointcloud from NormalEstimationIntegralImage
* [jsk_pcl_ros] torus should directs to origin always in TorusFinder
* [jsk_pcl_ros] Separate output library into 3 libraries in order to
  reduce memory usage of linking
* [jsk_pcl_ros] Fix README.md
* [jsk_pcl_ros] Publish geometry_msgs/PoseStamped and
  geometry_msgs/PointStamped from CentroidPublisher
* [jsk_pcl_ros] Fix coding style of CentroidPublisher
* [jsk_pcl_ros] Support spherical projection model in BorderEstimator
* Merge remote-tracking branch 'refs/remotes/origin/master' into range-image
* [jsk_pcl_ros] Support laser model in BorderEstimator and update document
* depth_calibration tutorial with link markup
* add depth calibration tutorial
* add depth calibration tutorial
* Merge pull request `#687 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/687>`_ from garaemon/cached-particle-filter
  [jsk_pcl_ros] Cache result o nearest-negihbor search
* [jsk_pcl_ros] Cache result o nearest-negihbor search
* Merge remote-tracking branch 'refs/remotes/origin/master' into 2d-reject
* [jsk_pcl_ros] Check direction of detected stick and hint line in 2-D image coordinate
* Fix license: WillowGarage -> JSK Lab
* Contributors: Ryohei Ueda, Yuto Inagaki, JSK Lab Member, Yu Ohara, Xiangyu Chen

0.2.3 (2015-02-02)
------------------
* [jsk_pcl_ros] Add ~min_inliers and ~cylinder_fitting_trial parameter to
  try cylinder fitting severeal times in HintedStickFinder
* [jsk_pcl_ros] Implement utility function to generate cylinder marker
  from cylinder object
* [jsk_pcl_ros] FIx mis-publishing of coefficients of HintedStickFInder
* [jsk_pcl_ros, jsk_perception] Move mask image operation to jsk_perception
* [jsk_pcl_ros] Publish inliers and coefficients from HintedStickFinder
* Remove rosbuild files
* [jsk_perception] Add DilateMaskImage
* Contributors: Ryohei Ueda

0.2.2 (2015-01-30)
------------------
* [jsk_pcl_ros] Add HintedStickFinder to detect stick with human interfaction
* Contributors: Ryohei Ueda, Kei Okada

0.2.1 (2015-01-30)
------------------
* Merge pull request #672 from k-okada/add_image_view2
  add image_view2
* [jsk_pcl_ros] add jsk_recognition_msgs to catkin_package:DEPEND
* [jsk_pcl_ros] Add HintedStickFinder to detect stick with human interfaction

0.2.0 (2015-01-29)
------------------

0.1.34 (2015-01-29)
-------------------
* support both yaml 0.3.0(hydro) and yaml 0.5.0(indigo)
* [jsk_pcl_ros] Fixed serious bug to detect points near from polygon
* use this to call methods, I need this to compile on indigo, but not sure if this really works, please check if this is correct @YuOhara, @garaemon
* depending on cv_bridge is recommended, see http://wiki.ros.org/indigo/Migration#OpenCV
* [jsk_pcl_ros] Update document and python script to use jsk_recognition_msgs
* [jsk_pcl_ros] Fix document indent and add image of HSIColorFilter
* [jsk_pcl_ros] Add documentation about RGBColorFilter and HSVColorFilter
* Fix unchanged path to message header
* [jsk_pcl_ros] Fix header location of find_object_on_plane.h
* [jsk_pcl_ros, jsk_perception] Move find_object_on_plane from
  jsk_perception to jsk_pcl_ros to make these packages independent
* [jsk_pcl_ros, jsk_perception] Use jsk_recognition_msgs
* [jsk_pcl_ros, jsk_perception, resized_image_transport] Do not include
  jsk_topic_tools/nodelet.cmake because it is exported by CFG_EXTRAS
* merge master
* [jsk_pcl_ros] Add image of TiltLaserListener to document
* add options for align box and change base_frame_id
* [jsk_pcl_ros] add ~not_publish_tf parameter to ParticleFilterTracking
* [jsk_pcl_ros] Refactor ParticleFilterTracking
* [jsk_pcl_ros] Optimize ReversedParticleFilter by not updating octree per
  each calculation
* [jsk_pcl_ros] Add *reversed* mode for ParticleFilterTracking and add
  sample to localize robot by tilt laser
* [jsk_pcl_ros] Fix documentation
  * Update picture of OrganizedMultiPlaneSegmentation
  * Fix indent
  * Fix AddColorFromImage picture
* [jsk_pcl_ros] Update ParticleFilterTracking document
* [jsk_pcl_ros] Increase initial number of particles to avoid SEGV
* Contributors: Ryohei Ueda, Kei Okada, JSK Lab Member

0.1.33 (2015-01-24)
-------------------
* [jsk_pcl_ros] Add magnify parameter to MultiPlaneExtraction
* [jsk_pcl_ros] Added several flags to toggle filtering in HintedPlaneDetector
* [jsk_pcl_ros] Update min-max value of min_height and max_height of MultiPlaneExtraction
* [jsk_pcl_ros] Publish indices from MultiPlaneExtraction
* [jsk_pcl_ros] Catch tf2::ExtrapolationException error in normal
  direction filter
* [jsk_pcl_ros] Add euclidean segmentation to hinted plane detector sample
* [jsk_pcl_ros] Close convex region
* [jsk_pcl_ros, jsk_perception] Fix CmakeList for catkin build. Check jsk_topic_tools_SOURCE_PREFIX
* update params for tracking
* [jsk_pcl_ros] AddPointIndices
* [jsk_pcl_ros]change border_estimator to publish indices instread of pointcloud
* [jsk_pcl_ros] Refactor HintedPlaneDetector
* [jsk_pcl_ros] Add density filtering to HintedPlaneDetector
* [jsk_pcl_ros] Supress warning message from OrganizedMultiPlaneSegmentation
* [jsk_pcl_ros] add ~overwrap_angle parameter to TiltLaserListener
* [jsk_pcl_ros] Add nodelet to convert geometry_msgs/PolygonStamped into
  mask image
* [jsk_pcl_ros] Initialize centroid value
* [jsk_pcl_ros] Check if a point is nan in ROIClipper
* [jsk_perception] Update HintedPlaneDetector with better algorithm.
* [jsk_pcl_ros] Supress warning message from NormalConcatenator
* [jsk_pcl_ros] Fix timestamp of pointcloud of TiltLaserListener and do
  not publish same pointcloud twice by TiltLaserListener
* [jsk_pcl_ros] add ROIToMaskImage and ROIToRect
* [jsk_pcl_ros] Add RectToMaskImage and MaskImageFilter to filter
  non-organized pointcloud by mask image
* standize codes around brackets
* clean codes in particle_filter_tracking
* add frame_id_decision
* [jsk_pcl_ros] implement mask image converters: MaskImageToROI and MaskImageToRect
* add tracking option that initialize first pose with BBox
* adding comments to pointcloud_screenpoint.launch and relatives
* [jsk_pcl_ros] Add TorusFinder
* [jsk_pcl_ros] update document about ROIClipper
* [jsk_pcl_ros] Fix ROIClipper and RectToROI to work
* [jsk_pcl_ros] Do not take nested lock of mutex in roi_cipper
* [jsk_pcl_ros] Support pointcloud filtering by ROI in ROIClipper and add
  converter from rectangle region into ROI
* [jsk_pcl_ros] nodelet to add color to pointcloud from image
* [jsk_pcl_ros] nodelet to add color to pointcloud from image
* add none result publisher when reference is empty
* [jsk_pcl_ros] Publish pose of matched template in LINEMOD
* Contributors: Ryohei Ueda, Hiroaki Yaguchi, JSK Lab Member, Yu Ohara, Yuto Inagaki

0.1.32 (2015-01-12)
-------------------
* add Torus.msg and TorusArrray.msg
* [jsk_pcl_ros, checkerboard_detector] Fix offset from checker board
* [jsk_pcl_ros] Use pcl::LINEMOD in LINEMODDetector for memory efficiency
* [jsk_pcl_ros] Use linemod class when training linemod template
* [jsk_pcl_ros] tune parameter of multi plane based object detection using
  spindle laser
* Contributors: Ryohei Ueda, Yuto Inagaki

0.1.31 (2015-01-08)
-------------------
* Merge pull request #563 from garaemon/no-indices-for-multi-plane-extraction
  [jsk_pcl_ros] Parameter to disable indices in MultiPlaneExtraction
* [jsk_pcl_ros] Do not use indices in MultiPlaneExtraction
* Merge pull request #562 from garaemon/add-plane-concatenator
  [jsk_pcl_ros] PlaneConcatenator: nodelet to concatenate near planes
* [jsk_pcl_ros] PlaneConcatenator: nodelet to concatenate near planes
* Merge pull request #561 from garaemon/add-clear-cache-service
  [jsk_pcl_ros] Add ~clear_cache service to TiltLaserListener
* [jsk_pcl_ros] Add ~clear_cache service to restart collecting
  laser data in TiltLaserListener
* [jsk_pcl_ros] Support multiple interest region in AttentionClipper
* [jsk_pcl_ros] Support initial pose of AttentionClipper
* [jsk_pcl_ros/LINEMODTrainer] Use wildcard in compressing data to
  generate ltm
* [jsk_pcl_ros] Multithread safe LINEMODTrainer by avoiding
  pcl::RangeImage non-thread safe initialization
* [jsk_pcl_ros] Do not publish range image (It's not stable under OpenMP)
  and use directory rather than filename when calling tar
* [jsk_pcl_ros] Train linemod with OpenMP and publish range image
  with color
* [jsk_pcl_ros] Utility launch file and scripts to training LINEMOD from
  bag file
* [jsk_pcl_ros] Add image for LINEMODTrainer documentation
* [jsk_pcl_ros] Decrease memory usage when training LINEMOD
* [jsk_pcl_ros] Sampling viewpoint to generate training data
  for LINEMOD
* [jsk_pcl_ros] Remove linemod rotation quantization
* [jsk_pcl_ros] Use triangle decomposition to check a point is inside
  or not of polygon
* [jsk_pcl_ros] Add picture of LINEMODDetector
* [jsk_pcl_ros] SupervoxelSegmentation: new nodelet to wrap
  pcl::SupervoxelClustering
* [jsk_pcl_ros] Refine Model by ICP in IncrementalModelRegistration
* [jsk_pcl_ros] Add simple icp service to ICPRegistration
* [jsk_pcl_ros] add utility launch file to capture training data from multisense
* [jsk_pcl_ros] Publish the number of samples from CaptureStereoSynchronizer
* [jsk_pcl_ros] Fix when ROI is outside of the image in AttentionClipper
* [jsk_pcl_ros] Fix when ROI is outside of the image in AttentionClipper
* Merge pull request #532 from garaemon/add-mask-image-to-point-indices
  [jsk_pcl_ros] Add MaskImageToPointIndices
* Merge pull request #531 from garaemon/add-incremental-pointcloud-registration
  [jsk_pcl_ros] IncrementalModelRegistration Add new nodelet to build full 3d model from sequentially captured pointcloud
* fix to compile on indigo #529
* [jsk_pcl_ros] MaskImageToPointIndices: add nodelet to convert mask image to point indices
* [jsk_pcl_ros] Add new nodelet to build full 3d model from
  sequentially captured pointcloud: IncrementalModelRegistration
* [jsk_pcl_ros] untabify icp_registration_nodelet.cpp
* [jsk_pcl_ros] update document of IntermittentImageAnnotator
* [jsk_pcl_ros] Storing pointcloud and publish pointcloud inside
  of ROI specified
* [jsk_pcl_ros] Visualize selected ROI as marker in IntermittentImageAnnotator
* [jsk_pcl_ros] Add ~rate parameter to throttle image publishing from IntermittentImageAnnotator
* add camera frame param to handle_estimator.l

0.1.30 (2014-12-24)
-------------------
* Publish specified ROI as PosedCameraInfo in IntermittentImageAnnotator
* Use TfListenerSingleton to get instance of tf::TransformListener
* Contributors: Ryohei Ueda

0.1.29 (2014-12-24)
-------------------
* Add document about IntermittentImageAnnotator
* [LINEMODDetector] Do not use small templates
* [CaptureStereoSynchronizer] Does not capture near samples
* Add IntermittentImageAnnotator to select ROI out of several snapshots
* [LINEMODDetector] Use glob to specify template files for linemod
* [LINEMODTrainer] Simulate samples rotating around z-axis
* Add projective ICP registration
* Write PCD file as binary compressed in LINEMODTrainer
* Load linemod training data from pcd and sqmmt files and use OpenMP
  to speed-up it
* Synchronize reference pointcloud and input pointcloud in icp registration
  to refine result of other recognition
* LINEMODDetector: add documentation and load template after setting
  parameters and publish the result of recognition as pointcloud
* Add LINEMODDetector and implement LINEMODTrainer and LINEMODDetector in
  one linemod_nodelet.cpp
* fix transform mistake
* Fix linemod template format. lmt is just a tar file of pcd and sqmm files
* rotate pose of box acoording to looking direction
* Add launch file to reconstruct 3d pointcloud from captured by CaptureStereoSynchronizer
* Add nodelet to train linemod
* Move multisense specific lines from capture.launch to capture_multisense_training_data.launch
* Added new nodelet to capture training data of stereo camera to
  jsk_pcl_ros and update launch files to capture training data of multisense
* Add new nodelet to generate mask image from PointIndices
* Clip Pointcloud and publish the indices inside of a box in AttentionClipper
* Added topic interface to specify the region by jsk_pcl_ros::BoundingBox
* add parameter to choose keeping organized
* Add utility launch file to resize pointcloud and fix initial value of
  use_indices_ in resize_points_publisher_nodelet.cpp
* Support pointclouds include nan in EuclideanClustering
* Remove diagnostic_nodelet.{cpp,h} and connection_based_nodelet.{cpp,h}
  of jsk_pcl_ros and use them of jsk_topic_tools
* Use jsk_topic_tools::ConnectionBasedNodelet in DepthImageError, EdgeDepthReginement, EdgebasedCubeFinder, EuclideanClusterExtraction and GridSampler
* add parameter
* print handle estimation
* use handle_estimator.l instead of nodelet version
* add euslisp handle estimator
* handle_estimator : change condition or to and
* Contributors: Ryohei Ueda, Yusuke Furuta, Chi Wun Au, Yuto Inagaki

0.1.28 (2014-12-17)
-------------------
* Publish attention region mask from AttentionClipper
* Add new nodelets: ROIClipper and AttentionClipper to control attention
  and ROI
* fix hsi_color_filter.launch bug
* Change default value of publish_tf and publish_clouds of ClusterPointIndicesDecomposer

0.1.27 (2014-12-09)
-------------------
* Add GDB argument to toggle xterm gdb hack
* changed default parametar for pub_tf false
* added args in launch not pub tf by cluster_decomposer
* Enable to create several hsi filters
* fixed bug in icp
* add param to set angle-divide-param for organized multi plange
* Fix coding style of DepthImageCreator:
  * remove hard tabs
  * add bsd header
* Use jsk_topic_tool's ConnectionBasedNodelet in DepthImageCreator
* Add example euslisp code for displaying BoundingBoxArray
* Fix typo in rgb filter comments
* changed some topics in icp always subscribe without subscribe method defined in connection_based_nodelet
* changet pointcloud_screen_point not to use jsconnection_based_nodelet
* Use jsk_topic_tools::ConnectionBasedNodelet in BilateralFilter,
  BorderEstimator, BoundingBoxFilter and so on
* Contributors: Ryohei Ueda, Shunichi Nozawa, Yu Ohara, Yuto Inagaki

0.1.26 (2014-11-23)
-------------------
* Install launch directory
* Contributors: Ryohei Ueda

0.1.25 (2014-11-21)
-------------------
* Add singleton class for tf::TransformListener
* python_sklearn -> python-sklearn, see https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml#L1264
* Merge remote-tracking branch 'origin/master' into add-more-parameter-for-calibration
  Conflicts:
  jsk_pcl_ros/launch/openni2_remote.launch
* Add uv_scale parameter to depth_calibration.cpp and update openni2_remote.launch
  to specify more parameter.

0.1.24 (2014-11-15)
-------------------
* Add default calibration file for openni2_remote.launch
* remove unneeded nodelet part
  change param
* added launch that calc plane with pr2_laser
* Fix polygon projection and confirm that snapit works
* Fix MultiPlaneExtraction initialization
* Update SnapIt to use topic interface and reimplement it only for snap on polygon
* Fix segv in collision checking
* Fix OrganizedMultiPlaneSegmentation indexing
* Update diagnostics aggregator settings for footstep_recognition
* Fix diagnostic information when there is no subscriber
* Suppress message from EnvironmentPlaneModeling
* Add document about MultiPlaneExtraction
* Check the pointer is correctly set to avoid SEGV
* Add normal direction filter based on Imu direction
* Update OrganizedMultiPlaneDetection documentation
* Add new nodelet: region growing based multiple plane detection
* use this->erase
* Add imu hint when running MultiPlaneSACSegmentation
* Add short documentation about OrganizedMultiPlaneSegmentation
* Update document about CentroidPublisher
* Add documentation about jsk_pcl/ClusterPointIndicesDecomposerZAxis
* Add moveit plugin to just filter pointcloud which belongs to robot
* Add nodelet to handle time range of rotating laser
* removed passthrough filter
* rename file name from error_visualize to pr2_pointcloud_error_visualizatoin
* Support cluster information in MultiplePlaneSACSegmentation and remove
  plane estimation from LineSegmentCollector
* restored codes slightly
* added icp_result_msgs and srvs
* change launch file path
* add launch files for visualizing calibration error
* Add nodelet to handle time range of rotating laser
* Fix Polygon::decomposeToTriangles if the original polygon is already a triangle
* Remove single_nodelet_exec.cpp.in
* Add documentation about ClusterPointIndicesDecomposer
* Add image to documentation of EuclideanClustering
* Add documentation about EuclideanSegmentation
* Add documentation about DepthImageCreator
* Add documentation about PointcloudScreenpoint
* Support specifying yaml file to calibrate depth image on openni2_remote.launch
* Format calibration model on DepthCalibration
* For precision requirement, use repr function when generating yaml file
  for depth image calibration
* Support quadratic model for u and v to calibrate depth image:
  1. Support quadratic-uv-quadratic and quadratic-uv-quadratic-abs model
  2. use SetDepthCalibrationParameter.srv to specify depth calibration parameter
* Downsize frequency map resolution and add --width and --height option to
  depth_error_calibration.py
* Update depth calibration program.
  1. Fix checkerboard_detector to publish correct corner point
  2. Calibrate depth_image rather than PointCloud
  3. Use matplotlib animation to visualize graph in depth_error_calibration.py
* support new model to calibrate kinect like sensor, which use absolute
  value respected to center coordinate of projectoin matrix
* Support quadratic-uv-abs model
* Add service file: DepthCalibrationParameter
* Add nodelet to apply calibration parameter to pointcloud. and add
  new model to calibrate: quadratic-uv
* Support quadratic function fitting in depth_error_calibration.py
* Add python script to calibrate depth error of depth sensors
* Merge remote-tracking branch 'refs/remotes/origin/master' into add-document-about-resize-points
  Conflicts:
  jsk_pcl_ros/README.md
* Add script to run logistic regression for depth error
* Add documentation about ResizePointCloud
* Merge remote-tracking branch 'refs/remotes/origin/master' into remove-color-category20-from-jsk-pcl-ros
  Conflicts:
  jsk_pcl_ros/include/jsk_pcl_ros/pcl_util.h
* Remove colorCategory20 from jsk_pcl_ros and use jsk_topic_tools' colorCategory20
* Fix syntax of README.md of jsk_pcl_ros
* Add documentation about ResizePointCloud
* Add documentation about typical messages defined in jsk_pcl_ros
* Extract multi planes out of collected segmented lines from laserrange finder
* add new nodelet: LienSegmentCollector
* Add LineSegmentDetector for LRF pointcloud
* Use dynamic reconfigure to specify several parameters for ParticleFilterTracking
* Support contiuous model building on EnvironmentPlaneModeling and add
  a launch file for footstep planning recogniton
* Add utitlity service interface to register completed maps
* Contributors: Kei Okada, Yuto Inagaki, JSK applications, Chi Wun Au, Ryohei Ueda, Yu Ohara

0.1.23 (2014-10-09)
-------------------
* Use pcl::EarClip to decompose polygon into triangles
* Complete gridmap with statically defined polygon
* Install nodelet executables
* Use jsk_topic_tools::readVectorParameter in ParticleFilterTracking
* Add BilateralFilter
* Decrease size of grid map to add 'padding'
* Add service to clear grid maps
* Add min-max threshold to filter polygons based on area on OrganizedMultiPlaneSegmentation
* EnvironmentPlaneModeling support building grid map without static
  polygon information
* delete models
* Fix env_server's mis posing of origin
* Force for planes to direct sensor origin in organized multi segmentation
* Support PointcloudDatabaseServer when running ICPRegistration
* Add PointCloudDatabaseServer
* Fix keypoints publisher compilation
* Subscribe topics as needed for almost all the nodelets
* Use ConnectionBasedNodelet for DelayPointCloud not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for ColorizeDistanceFromPlane not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for DelayPointcloud not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for ColorizeDistanceFromPlane not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for ColorHistogramMatcher not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for BoundingBoxFilter not to subscribe topics if the nodelet's publishers are not subscribed
* Use ConnectionBasedNodelet for ResizePointsPublisher not to subscribe
  topics if the nodelet's publishers are not subscribed
* Do not subscribe until any publish is subscribed on ColorFIlter and
  BorderEstimator
* Do not subscribe until any publisher is subscribed on
  ClusterPointIndicesDecomposer and add utlity class to handle connection
* Fix JointStateStaticFilter to use absolute diff when calculating
  time difference and add JointStateStaticFilter to organized_multi_plane_segmentation.launch
  if JOINT_STATIC_FILTER:=true
* Use refined plane information in recognition pipeline
* Add pr2_navigation_self_filter to organized_multi_plane_segmentation.launch
* Publish result of ICP as geometry_msgs::PoseStamped
* Add pcd model files for registration sample
* Use PLUGIN_EXPORT_CLASS instead of PLUGIN_DECLARE_CLASS
* Considering flipped initial pose on ICP registration
* Merge remote-tracking branch 'refs/remotes/origin/master' into use-boundingbox-information-to-compute-origin-of-icp-pointcloud
  Conflicts:
  jsk_pcl_ros/jsk_pcl_nodelets.xml
* Add new nodelet to transform pointcloud to make its origin equal to the
  pose of boundingbox and use bounding box information when running ICP
* Merge pull request `#307 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/307>`_ from garaemon/joint-state-static-pointcloud-filter
  JointStateStaticFilter
* Add Generalized ICP algorithm
* read voxel grid donwsample manager parameter
* Merge remote-tracking branch 'refs/remotes/origin/master' into garaemon-joint-state-static-pointcloud-filter
  Conflicts:
  jsk_pcl_ros/CMakeLists.txt
  jsk_pcl_ros/catkin.cmake
  jsk_pcl_ros/jsk_pcl_nodelets.xml
* Add new nodelet to pass pointcloud only if joint states is stable
* Support dynamic_reconfigure of ICPRegistration
* add new nodelet to align two pointcloud based on ICP algorithm
* Fix for plane segmentation results into only one plane
* Add new nodelet 'PlaneReasoner' to segment wall/ground
* Resize pointcloud and images in openni_remote.launch
* Fix topic relaying of openni_remote for openni_launch on hydro
* Add new nodelet to filter organized pointcloud based on x-y index rather
  than 3-D position value.
* Contributors: Ryohei Ueda, Yusuke Furuta, Yuto Inagaki

0.1.22 (2014-09-24)
-------------------

0.1.21 (2014-09-20)
-------------------
* Add utility methods for 2-D geometry
* Add new nodelet to filter bounding box array
* Check align axis before aligning boundingbox in ClusterPointIndicesDecomposer
* Add diagnostic information to EuclideanClusteringExtraction
* Add diagnostic information to MultiPlaneExtraction
* Add processing frame id information to PlaneRejector's diagnostic
* Add diagnostic information to ClusterPointIndicesDecomposer
* Add diagnostics to PlaneRejector
* Add more diagnostics to OrganizedMultiPlaneSegmentation and fix global
  hook for ConvexHull
* Contributors: Ryohei Ueda

0.1.20 (2014-09-17)
-------------------
* Not use inliers to colorize pointcloud based on distance from planes
* Add check to be able to make convex or not on ColorizeDistanceFromPlane
  and OrganizedMultiPlaneSegmentation
* add ~use_normal to use noraml to segment multi planes
* add new nodelet to segment multiple planese by applying RANSAC recursively
* Contributors: Ryohei Ueda

0.1.19 (2014-09-15)
-------------------

0.1.18 (2014-09-13)
-------------------
* Subscribe PolygonArray message to build ConvexPolygon in ColorizeDistanceFromPlane
* Introduce global mutex for quick hull
* Fix coloring bug and add ~only_projectable parameter to visualize the
  points only if they can be projected on the convex region
* Add use_laser_pipeline argument to laserscan_registration.launch to
  toggle include laser_pileline.launch of jsk_tilt_laser or not
  Add new utility for diagnostics: addDiagnosticInformation
* Supress output from resize_points_publisher
* ROS_INFO -> NODELET_DEBUG in VoxelGridDownsampleManager
* New utilify functoin for diagnostic: addDiagnosticInformation.
  It's a simple function to add jsk_topic_tools::TimeAccumulator to
  diagnostic_updater::DiagnosticStatusWrapper.
* Colorize pointcloud according to the distance from nearest plane
* Use template functions to convert tiny type conversions
* Refine the result of connecting small multi planes in OrganizedMultiplaneSegmentation
* add hsv coherence to particle_fitler_tracker
* change color_histogram showing methods with reconfigure
* visualize color_histogram coefficience
* add new nodelet: EdgebasedCubeFinder
* use colorCategory20 function to colorize pointcloud in ClusterPointIndicesDecomposer
* visualizing connection of planes with lines in OrganizedMultiPlaneSegmentation
* use rosparam_utils of jsk_topic_tools in StaticPolygonArrayPublisher
* Contributors: Ryohei Ueda, Wesley Chan, Yu Ohara

0.1.17 (2014-09-07)
-------------------
* add laser_registration.launch
* Contributors: Yuki Furuta

0.1.16 (2014-09-04)
-------------------
* bugfix: add depth_image_creator to jsk_pcl_nodelet on catkin.cmake
* a launch file for stereo camera using pointgrey
* Publish ModelCoefficients from EdgeDepthRefinement
* Add new nodelet to detect parallel edge
* Remove duplicated edges according to the line coefficients in
  EdgeDepthRefinement
* do not use EIGEN_ALIGNED_NEW_OPERATOR and use onInit super method on
  PointcloudScreenpoint
* Remove several unused headers from ParticleFilterTracking
* not compile OrganizedEdgeDetector on groovy
* add a new nodelet to refine edges based on depth connectivity
* Detect straight edges from organized pointcloud
* toggle edge feature by rqt_reqoncifugre in OrganizedEdgeDetector
* add new nodelet: OrganizedEdgeDetector, which is only available with
  latest PCL
* Do not include header of cloud viewer in region_growing_segmentation.h
* Add more diagnostic information to OrganizedMultiPlaneSegmentation
* downsample rgb as well as pointcloud in openni2_remote.launch
* add new nodelet: BorderEstimator
* Contributors: Ryohei Ueda, Yuki Furuta

0.1.14 (2014-08-01)
-------------------
* add bounging box movement msg
* Contributors: Yusuke Furuta

0.1.13 (2014-07-29)
-------------------
* add include of pcl_util.h to OrganizedMultiPlaneSegmentation
* use jsk_topic_tools::TimeAccumulator instead of
  jsk_pcl_ros::TimeAccumulator in jsk_pcl_ros
* new class to check connectivity; VitalChecker
* fixing the usage of boost::mutex::scoped_lock
* use Eigen::Vector3f as a default type in geo_util classes
* Contributors: Ryohei Ueda

0.1.12 (2014-07-24)
-------------------
* Merge pull request `#210 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/210>`_ from aginika/add-remove-nan-funtion-line
  Add remove nan funtion line
* prevent nan pointcloud error with inserting removeNan function in ParticleFilterTracking
* fix environment modeling and changed api to lock/unlock environment
* remove border region from environment model
* publish diagnostic information from OrganizedMultiPlaneSementation
* take the average of plane coefficients to be combined in EnvironmentPlaneModeling
* wait transform before transforming in PolygonArrayTransformer
* convert convex line information into grid cell before computing grid cell
* fix normalization of the normal when creating Polygon object
* catch more exceptions in TfTransformPointCloud nodelet
* Supress debug message from ColorHistogramMatcher
* fill x-y-z field to publish correct pose of the pointcloud from ColorHistogramMatcher
* publish the pose of the best matched candidate in ColorHistogramMatcher
* publish selected handle pose
* publish u, v, true_depth and observed_depth
* fix the order of Mat::at
* add two nodelets (DelayPointCloud and DepthImageError) to jsk_pcl_ros
  and publish u/v coordinates of the checkerboard from checkerboard_detector.
  * DepthImageError is just a skelton yet.
  * DelayPointCloud re-publishes pointcloud with specified delay time.
  * publish u/v coordinates from checkerboard_detector.
  * frame_id broadcasted from objectdetection_tf_publisher.py is configurable
* copy the header of the input cloud to the output cloud in SelectedClusterPublisher
* Contributors: Ryohei Ueda, Eisoku Kuroiwa, Yusuke Furuta, Yuto Inagaki

0.1.11 (2014-07-08)
-------------------

0.1.10 (2014-07-07)
-------------------
* compute distance based on Polygon-to-ConvexCentroid in order to identify
  the grid maps
* remove debug code in PolygonArrayTransformer
* use Plane class to compute transformation of coefficients
* statical voting and rejection to the grid map to remove unstable
  recognition result
* support appending of GridMap in time series in EnvironmentPlaneModeling
* measure time to compute polygon collision in EnvironmentPlaneModeling
* add a nodelet to concatenate PolygonStamped
* publish polygon synchronized with ~trigger message
* new utility class to measure time
* change default camera name
* build and publish grid map always on EnvironmentPlaneModeling
* add launch file for openni
* Contributors: Ryohei Ueda, Yusuke Furuta

0.1.9 (2014-07-01)
------------------
* publish the result of grid modeling as SparseOccupancyGridArray
* compute segmented cloud's distance to polygon based on convex polygon assumption
* add new parameter to dynamic_reconfigure of EnvironmentPlaneModeling
* Contributors: Ryohei Ueda

0.1.8 (2014-06-29)
------------------
* add min_indices parameter to ignore the grid which does not include
  enough points.
* add throttle for bounding box visualization in
  organized_multi_plane_segmentation.launch.
  Because it may be too fast to see...
* add ~publish_tf=false to several nodelets in organized_multi_plane_segmentation.launch
* fix typo of launch file
* run ColorHistogramMatcher with GridSampler
* implement GridSampler
* find object based on hsv color histogram of the pointcloud
* implement simple handle detector to grasp
* refactor cluster decomposer class
  run PCA to compute orientation of bounding box
* run PCA to compute bouding box
* fix segmentation fault
* estimate occlusion in EnvironmentPlaneModeling
* fix several bags for plane-based environment modeling
* fix the header of the output of the estimation of occlusion
* do not compute transformation if no points are available
* does not publish pointclouds if transformation failed
* merge remote branch origin/master
* fulfill occluded reagion with pointcloud by OccludedPlaneEstimator
* debug and substitute stamp value to header/stamp
  add cloth classification sample
* only make will be executed on hydro
* fix typo: oclusoin -> occlusion
* add new nodelet: EnvironmentPlaneModeling
* use pcl::PointXYZRGB rather than pcl::PointXYZRGBNormal
* add normal estimation to organized multi plane segmentation
* d varaible of the normal should be transformed correctly by PolygonArrayTransfomer.
  fix transformation compuation to normalize d parameter
* add depent tag to ml_classifiers
* add more rosparameters to ParticleFilterTracking
* add MACHINE and GDB argument
* add program to compute color histogram (rgb and hsv color space)
* add cloth classification sample
* change the namespace of the topics to use tracking.launch from the other launch files
* add OcludedPlaneEstimator nodelet to estimate the ocluded planes
* new nodelet to transform PolygonArray and ModelCoefficientsArray
* add nodelet to publish static jsk_pcl_ros/PolygonArray with timestamp
  synchronized with the pointclouds
* Contributors: Ryohei Ueda, Yusuke Furuta, Masaki Murooka, Yuto Inagaki

0.1.7 (2014-05-31)
------------------
* new nodelet to reject the plane which does not satisfy the threshold
  about normal direction
* simplyfy tracking and add update with msg function

0.1.6 (2014-05-30)
------------------

0.1.5 (2014-05-29)
------------------
* add new nodelet to publish the points of the cluster selected by
  jsk_pcl_ros/Int32Stamped.
  this nodelet is supposed to be used with jsk_interactive_marker/bounding_box_marker
* align the boxes to the nearest plane
* add new parameter publish_clouds to ClusterPointIndicesDecomposer
  to disable publishing decomposed pointclouds
* add new message: BoundingBox and BoundingBoxArray and publish
  BoundingBoxArray from ClusterPointIndicesDecomposer
* use enum to select estimation method of NormalEstimationIntegralImage
* add launch and rviz file for subway bagfiles
* remove IndiceArray.msg, which are not used any more
* publish empty result if segmentation failed
* update the default parameters
* use PointXYZRGBNormal rather than PointXYZ nor Normal to speed up
  pointcloud conversion between ROS <-> PCL
* for realtime organized multi plane segmentation, add optimization flag
* add curvature veature
* comment in again and remove centroid publisher
* fix conflicts
* fix the size of the AABB published from ClusterPointIndicesDecomposer
* update launch file for OrganizedMultiPlaneSegmentatoin.
  introduce several arguments.
  add several HzMeasure to measure the speed of the processing
* add new nodelet: NormalEstimationIntegralImage
* add new nodelet: NormalEstimationIntegralImage
* add dynamic reconfigure to MultiPlaneExtraction
* commnet out hsv-limit and remove centroid publisher
* use ExtractPolygonalPrismData class to extract the pointcloud ON the planes
* add new class: MultiPlaneExtraction to extract the points which does not
  belong to the planes. However it's not so stable and efficient now
* publish the result of the clustring as polygon with convex hull
  reconstruction. and publish the result of the plane estimation as ModelCoefficientsArray.
* implement connectiong of the planes segmented by organized multi planse segmentation
* output the segmentation as PolygonArray as the result of
  OrganizedMultiPlaneSegmentation
* delete unneeded files
* rearrange many launch files , rviz files and add sample for rosbags
* add argument for camera_info url
* fix for groovy
* does not compile region growing segmentation on groovy
* publish colorized points from cluster point indices decomposer
* does not compile on groovy
* does not compile region growing segmentation on groovy
* implement OrganizedMultiPlaneSegmentation
* add new nodelet: RegionGrowingSegmentation based
  on pcl::RegionGrowingSegmentation class
* add pcl_ros/NormalConcatenater nodelet.
  it retrieves PointXYZRGB from ~input and Normal from ~normal and
  concatenate them into ~output as PointXYZRGBNormal
* update index.rst
* delete wrong commited files
* update README and arrage some launch files directory
* fix for groovy
* use pclpointcloud2
* add sample_610_clothes.launch
* remove the sample launch files for non-used color converter and color filter
* rename rgb_color_filter.cpp and rgb_color_filter.h
  to rgb color_filter.cpp and color_filter.h.
* use the lines rather than cube to visualize bounding box
* add hsi_color_filter executable
* implement resize_points_publisher w/o filter class.
  remove nonused files such as color_filter, color_converter and so on.
* add marker to display the result of the clustering as bounding boxes
* publishes tf frames to the center of the clusters
* add euclidean clustering, decomposer and zfilter
* add filter.cpp to jsk_pcl_ros on rosbuild. because resize points publisher requires it.
  this is a hotfix, so I will re-implement that nodelet w/o filter.cpp
* support groovy and pcl 1.6
* compile cluster_point_indices_decomposer and cluster_point_indices_decomposer_z_axis on catkin
* compile euclidean_cluster_extraction_nodelet.cpp on catkin
* add add HSI Color filter
* rgb_color_filter.launch: add comment and launch centroid_publisher as default
* catch tf exception
* remove redundant declaration of TransformBroadcaster
* remove redundant declaration of TransformBroadcaster
* update README and add centroid related files
* do not run dynamic reconfigure callback and topic callback symultenously
* support ~indices topic to specify indices vector of the points and refactor codes
* re-implement RGBColorFilter as simpler class
* add centroid_publisher to catkin
* add tracking rviz config
* delete unneeded line in tf_transfomr_cloud.launch
* add tf transform cloud launch and rviz
* add octree_change_detector.launch
* add group tag to create local scope to remap several topics in openni2.launch
* To update README, add explanation to tracking , octree and tf cloud
* relaying camera_info under camera_remote namespace
* add tf transform nodelet
* make paritcal_filter_tracking_nodelet publish tracked object tf trasnformation
* add two launch files to run openni on remote machine
* add octree_change_detector
* Contributors: Ryo Terasawa, Chan Wesley, Shunichi Nozawa, Yuto Inagaki, Masaki Murooka, Ryohei Ueda, Yohei Kakiuchi, Yusuke Furuta, Kei Okada

0.1.4 (2014-04-25)
------------------
* fixed compile error jsk_pcl_ros
* Contributors: Ryohei Ueda, Kei Okada, Yuto Inagaki

0.1.3 (2014-04-12)
------------------
* add depends to visualization_msgs
* delete lines for refactoring the tracking
* add RGB color
* fill point_cloud field
* Contributors: Ryohei Ueda, Kei Okada, Yuto Inagaki

0.1.2 (2014-04-11)
------------------
* use find_moduel to check catkin/rosbuild to pass git-buildpackage
* Contributors: Kei Okada
* add CallPolygon.srv for `jsk-ros-pkg/jsk_smart_apps#17 <https://github.com/jsk-ros-pkg/jsk_smart_apps/issues/17>`_
* Contributors: Yuto Inagaki

0.1.1 (2014-04-10)
------------------
* add depend_tag for pcl_conversions and not needed tags
  delete not needed tags
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: catch runtime error in order to ignore error from tf and so on
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: use SlicedPointCloud in VoxelGridDownsampleDecoder and use NODELET_** macros
  instead of ROS_** macros
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_:  use SlicedPointCloud in VoxelGridDownsampleManager
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: add new message for VoxelGrid{Manager,Decoder}: SlicedPointCloud.msg
* replacing image_rotate namespace with jsk_pcl_ros because of porting
* fix package name of dynamic reconfigure setting file
* use ROS_VERSION_MINIMUM
* use TF2_ROS_VERSION instead of ROS_MINIMUM_VERSION macro
* use tf2::BufferClient on groovy
* add cfg file for image_rotate dynamic reconfigure
* porting image_rotate_nodelet from image_pipeline garamon's fork.
  this version of image_rotate supports tf2 and nodelet.
* add rosdepend to prevent pointcloud_screenpoint_nodelet error
* use jsk nodelet mux for pcl roi
* add arg to set nodelet manager name
* use the same nodelet manager as openni
* `#20 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/20>`_: implement PointCloudThrottle and ImageMUX, ImageDEMUX and ImageThrotle
* add sensor_msgs dependency to message generation
* Merge remote-tracking branch 'refs/remotes/garaemon/add-message-dependency-to-jsk-pcl-ros' into garaemon-avoid-roseus-catkin-bug
  Conflicts:
  jsk_pcl_ros/catkin.cmake
* change the location of generate_messages and catkin_package of jsk_pcl_ros
* add sensor_msgs depdendency to jsk_pcl_ros's message generation
* `#8 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/8>`_: remove delay pointcloud nodelet
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: remove unused comment
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: remove unused cpp source codes, now they are automatically generated from single_nodelet_exec.cpp.in
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: automatically generate the single nodelet programs on rosbuild
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: rename resize_points_publisher to resize_points_publisher_nodelet according to naming convention
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: fix endmacro syntax
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: automatically generate cpp codes in catkin build
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: add quotes to the template file
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: add a template file to run single nodelet
* add pcl_conversions to jsk_pcl_ros
* add eigen_conversions to jsk_pcl_ros dependency
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: specify package.xml by fullpath
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: add pcl to dependency if distro is groovy
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: pcl is not a catkin package
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: fix if sentence order
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: depend pcl catkin package in groovy
* listed up nodelets provided by jsk_pcl_ros
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: removed icp_server, it's just a sample program
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: remove LazyConcatenater and PointcloudFlowrate from CMakeLists.txt
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: remove LazyConcatenater and PointcloudFlowrate from jsk_pcl_nodelets.xml
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: removed LazyConcatenater and PointcloudFlowrate
* fix depend package -> rosdep name
* add keypoints publisher; first supported only nerf
* add code for using GICP if using hydro
* add PolygonArray.msg for catkin build system
* adding header
* adding more nodelet modules for catkin
* adding CallSnapIt.srv
* add tf topic name parameter
* add pcl roi launch files
* add base_frame parameter in voxel_grid_downsample
* adding special message for polygon array
* adding hinted plane detector to xml nodelet list
* enable use_point_array of screenpoint
* add include <pcl_conversions/pcl_conversions.h> for groovy
* use pcl_conversions for hydro, see http://wiki.ros.org/hydro/Migration#PCL
* fix wrong commit on
* forget to commit, sorry
* add SnapItRequest to add_message_files
* adding sample for hinted plane detector
* adding HintedPlaneDetector and pointcloudScreenpoint supports converting array of 2d points into 3d
* adding HintedPlaneDetector and pointcloudScreenpoint supports converting array of 2d points into 3d
* publishing marker as recognition result
* implemented snapit for cylinder model
* adding height field
* adding cylinder parameters
* supporting cylinder model fitting
* fix for groovy with catkin
* setting axis when snap to the plane
* fixing transformation concatenation
* adding new module: SnapIt
* fix issue `#268 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/268>`_, run sed only when needed
* does not publish if the grid is empty
* change the default value
* change the default value
* adding initial ROI
* adding initial ROI
* not cahing old points
* supporting the change of the voxel num
* supporting the change of the voxel num
* supporting the change of the voxel num
* supporting the change of the voxel num
* not remove previous pointcloud as long as possible
* change the default value to 300
* supporting frame_id
* using tf
* adding decoder for voxel grid downsample manager
* adding message
* adding voxel grid downsample manager
* adding voxel_grid_downsample_manager
* supporting dynamic reconfigure
* adding lazy concatenater
* adding lazy concatenate sample
* adding lazy_concatenater
* debug RGBColorFilter and HSVColorFIlter for hydro
* adding pointcloud_flowrate nodelet skelton
* adding pointcloud_flowrate nodelet skelton
* compile pointcloud_flowrate executable
* executable to run pointcloud_flowrate
* tracking.launch change to tracking_hydro.launch and tracking_groovy.launch
* add load_manifest for rosbuild
* fix filtering range when min value is grater than max value
* fix filter name
* add rgb filter
* add mutex::scoped_lock in particle_filter_tracking
* debug in renew_tracking.py ROS_INFO -> rospy.loginfo
* add scripts/renew_trakcing.py launch/tracking.launch
* use SetPointCloud2
* add particle filter trackig node/nodelt with SetPointCloud2.srv
* fix pointcloud_scrennpoint.cpp to use jsk_pcl_ros -> jsk_pcl, by k-okada
* enable respawning
* add depends to pcl_msgs
* adding icp server
* adding TOWER_LOWEST2
* support both catkin/rosbuild
* update catkin makefile, add _gencpp, _gencfg
* support both catkin/rosbuild
* add_dependences to jsk_pcl_ros_gencpp
* pcl -> pcl_msgs for pcl-1.7 (hydro), but use sed to force change pcl/ namespace for groovy
* hydro migration, pcl 1.7 is independent from ros, see http://wiki.ros.org/hydro/Migration
* use USE_ROSBUILD for catkin/rosbuild environment
* starting with the middle tower
* fixing typo
* fixing typo
* using positoin from /origin, instead of from robot frame id
* added code for running centroid_publishers to publish segmented point cloud centroids
* update the position parameter for the demo
* fixing the rotatio of camera
* update the index of tower, plate, using enum in srv
* adding service to move robot with just index
* update the parameter and the axis
* fix to move robot to the goal tower
* update to run with eus ik server
* resolve position of each tf
* set the quality of the mjpeg server 100
* fixing message of the modal of alert
* block the tower already having plates
* adding debug message
* adding empty function to move robot
* adding graph
* adding service type to move robot
* smaller fonts
* adding cluster num on debug layer
* adding the number of the clusters
* update
* update the message
* adding more states for hanoi-tower
* small fixes
* adding service to pickup tower
* adding text shadow
* click detection by service call
* cenrerize button
* adding help modal
* track the window size
* adding html to redirect to tower_detecct_viewer
* centerize the image
* centerize the image
* adding state
* introducing state machine
* detecting clicked cluster
* using tower_detect_viewer_server
* providing a class
* adding some web related files
* using filled flag
* update params for lab room
* specifying tf_timeout of image_view2
* not subscribing topic to refer timestamp
* fixing header timestamp
* using some topic to refer timestamp
* supporting marker id
* update
* update topic to use image_view2's image
* fixing draw_3d_circle
* add script to draw circle on image_view2
* using location.hostname for the IP address
* adding www directory for tower_detect brawser viewer
* adding a launch file to launch mjpeg_server
* adding CentroidPublisher
* empty CentroidPublisher class
* implementing z axis sorting
* more effective implementation
* more information about resetting tracking
* fixing registration parameter
* adding nodelet skelton cpp
* adding cluster_point_indices_decomposer_z_axis.cpp
* adding sortIndicesOrder as preparation to customize ordering technique
* adding new nodelet ClusterPointIndicesDecomposer
* adding more methods
* adding skelton class to decompose ClusterPointIndices
* adding license declaration
* adding launch file to examin euclidean segmentation
* fixing label tracking
* refactoring
* refactoring
* refactoring
* supporting label_tracking_tolerance
* refactoring
* implementing labeling tracking
* calculate distance matrix
* adding one more color
* refactoring
* fixing compilation warning
* calculate centroids at the first frame
* fixing indentation
* using static colors to colorize clustered pointclouds
* removing noisy output
* removing invalid comments
* supporting dynamic reconfigure for euclidean clustering
* fixing rotation
* adding /origin and /table_center
* adding two lanch files
* adding top level launch
* openni.launch with depth_registered=true
* fix missing dependancy
* update hsv_color_filter.launch
* add USE_REGISTERER_DEPTH argument to pointcloud_screenpoint.launch
* remove env-loader (localhost do not need env-loader)
* update parameter use_point false -> true
* add same parameters to not USE_VIEW
* fix strequal ROS_DISTRO env
* use ROS_Distributions instead of ROS_DISTRO for electric
* fix for electric
* add USE_SYNC parameter to pointcloud_screenpoint.launch
* update pointcloud_screnpoint.launch
* merged image_view2/points_rectangle_extractor.cpp to pointcloud_screenpoint
* add EuclideanClustering [`#119 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/119>`_]
* copy pcl_ros/filters/filter to jsk_pcl_ros directory due to https://github.com/ros-perception/perception_pcl/issues/9, [`#119 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/119>`_]
* add catkin.cmake, package.xml for groovy, remove nodelt depends on pcl_ros::Filter https://github.com/ros-perception/perception_pcl/issues/9
* fix description comment
* remove cv_bridge
* add sample code for using
* add lisp code for using pointcloud in roseus
* use tf::Quaternion instead of btQuaternion
* comment out pcl_ros/features/features.cpp
* libtbb -> tbb , see rosdep/base.yaml
* change rodep name:libtbb to tbb
* update index.rst,conf.py by Jenkins
* fix: high load of screenpoint
* fix: change dynamic config
* fix: variable range of hue
* delete obsolated files
* rewrite color_filter and color_filter_nodelet for PCL 1.5 and later
* update sample for color_filter
* update index.rst,conf.py by Jenkins
* changed arg setting for launch from pr2.launch using mux
* update index.rst,conf.py by Jenkins
* fix: for using pcl_ros/feature class
* changed kinect's name from camera to openni
* fix: depth_image_creator added to nodelet
* use machine tag with env-loader
* comment out old pcl modules
* remove machine tag, which is not used
* fix for compiling fuerte and electric
* fix row_step and is_dense variables for resized point cloud
* added service of switching topic for depth_image_creator
* update index.rst,conf.py by Jenkins
* outout launchdoc-generator to build directry to avoid svn confrict
* remove jskpointcloud dependency from jsk_pcl_ros
* copy depth_image_creator from unreleased
* add jsk_pcl_ros (copy from unreleased repository)
* Contributors: Haseru Chen, Youhei Kakiuchi, Yuki Furuta, Kei Okada, Yuto Inagaki, Chen Wesley, Kazuto Murase, Ryohei Ueda
