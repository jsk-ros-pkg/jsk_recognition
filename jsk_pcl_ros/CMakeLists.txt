# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
set(PCL_MSGS pcl_msgs) ## hydro and later

find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_utils jsk_pcl_ros_utils
  dynamic_reconfigure pcl_ros nodelet message_generation genmsg
  ${PCL_MSGS} sensor_msgs geometry_msgs jsk_recognition_msgs
  eigen_conversions tf_conversions tf2_ros tf
  image_transport nodelet cv_bridge
  jsk_topic_tools
  image_geometry
  jsk_footstep_msgs
  interactive_markers
  laser_assembler
  octomap_server
  octomap_ros
  octomap_msgs
  kdl_parser
  kdl_conversions
  )
find_package(PkgConfig)
pkg_check_modules(ml_classifieres ml_classifiers QUIET)
# only run in hydro
find_package(PCL REQUIRED)
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# For kinfu
find_package(PCL QUIET COMPONENTS gpu_kinfu_large_scale)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
IF(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
ENDIF()

add_service_files(FILES SwitchTopic.srv
  UpdateOffset.srv
  SnapFootstep.srv
  SetDepthCalibrationParameter.srv
  TransformScreenpoint.srv
  CheckCircle.srv
  RobotPickupReleasePoint.srv
  TowerPickUp.srv
  EuclideanSegment.srv
  TowerRobotMoveCommand.srv
  CallSnapIt.srv CallPolygon.srv
  EnvironmentLock.srv
  PolygonOnEnvironment.srv
  ICPAlignWithBox.srv
  ICPAlign.srv
  CheckCollision.srv)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/VoxelGridLargeScale.cfg
  cfg/PlaneSupportedCuboidEstimator.cfg
  cfg/InteractiveCuboidLikelihood.cfg
  cfg/ExtractParticlesTopNBase.cfg
  cfg/HeightmapMorphologicalFiltering.cfg
  cfg/HeightmapConverter.cfg
  cfg/HeightmapToPointCloud.cfg
  cfg/HeightmapTimeAccumulation.cfg
  cfg/GeometricConsistencyGrouping.cfg
  cfg/UniformSampling.cfg
  cfg/BorderEstimator.cfg
  cfg/HintedStickFinder.cfg
  cfg/HintedPlaneDetector.cfg
  cfg/TorusFinder.cfg
  cfg/NormalDirectionFilter.cfg
  cfg/RegionGrowingMultiplePlaneSegmentation.cfg
  cfg/LineSegmentCollector.cfg
  cfg/LineSegmentDetector.cfg
  cfg/ParticleFilterTracking.cfg
  cfg/BilateralFilter.cfg
  cfg/ICPRegistration.cfg
  cfg/OrganizedPassThrough.cfg
  cfg/EuclideanClustering.cfg
  cfg/HSIColorFilter.cfg
  cfg/RGBColorFilter.cfg
  cfg/ImageRotate.cfg
  cfg/RegionGrowingSegmentation.cfg
  cfg/OrganizedMultiPlaneSegmentation.cfg
  cfg/MultiPlaneExtraction.cfg
  cfg/NormalEstimationIntegralImage.cfg
  cfg/EnvironmentPlaneModeling.cfg
  cfg/ColorHistogramMatcher.cfg
  cfg/ClusterPointIndicesDecomposer.cfg
  cfg/GridSampler.cfg
  cfg/OrganizedEdgeDetector.cfg
  cfg/EdgeDepthRefinement.cfg
  cfg/ParallelEdgeFinder.cfg
  cfg/EdgebasedCubeFinder.cfg
  cfg/MultiPlaneSACSegmentation.cfg
  cfg/BoundingBoxFilter.cfg
  cfg/ResizePointsPublisher.cfg
  cfg/LINEMODDetector.cfg
  cfg/SupervoxelSegmentation.cfg
  cfg/FeatureRegistration.cfg
  cfg/FisheyeSphere.cfg
  cfg/MovingLeastSquareSmoothing.cfg
  cfg/OctreeVoxelGrid.cfg
  cfg/OctreeChangePublisher.cfg
  cfg/ExtractTopPolygonLikelihood.cfg
  cfg/Kinfu.cfg
  )

find_package(OpenCV REQUIRED core imgproc)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
macro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_nodelet_sources jsk_pcl_nodelet_executables)
endmacro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

macro(jsk_pcl_util_nodelet_deprecated _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_util_nodelet_deprecated_sources jsk_pcl_util_nodelet_deprecated_executables)
endmacro(jsk_pcl_util_nodelet_deprecated _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

if ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O0")
else ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O2 -g")
endif ($ENV{TRAVIS_JOB_ID})

# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
jsk_pcl_nodelet(src/pointcloud_screenpoint_nodelet.cpp "jsk_pcl/PointcloudScreenpoint" "pointcloud_screenpoint")
jsk_pcl_util_nodelet_deprecated(src/normal_flip_to_frame_nodelet.cpp "jsk_pcl/NormalFlipToFrame"
  "normal_flip_to_frame")
jsk_pcl_nodelet(src/particle_filter_tracking_nodelet.cpp "jsk_pcl/ParticleFilterTracking" "particle_filter_tracking")
jsk_pcl_nodelet(src/voxel_grid_downsample_manager_nodelet.cpp "jsk_pcl/VoxelGridDownsampleManager" "voxel_grid_downsample_manager")
jsk_pcl_nodelet(src/voxel_grid_downsample_decoder_nodelet.cpp "jsk_pcl/VoxelGridDownsampleDecoder" "voxel_grid_downsample_decoder")
jsk_pcl_nodelet(src/snapit_nodelet.cpp "jsk_pcl/Snapit" "snapit")
jsk_pcl_nodelet(src/keypoints_publisher_nodelet.cpp "jsk_pcl/KeypointsPublisher" "keypoints_publisher")
jsk_pcl_nodelet(src/hinted_plane_detector_nodelet.cpp "jsk_pcl/HintedPlaneDetector" "hinted_plane_detector")
jsk_pcl_util_nodelet_deprecated(src/centroid_publisher_nodelet.cpp "jsk_pcl/CentroidPublisher" "centroid_publisher")
jsk_pcl_nodelet(src/moving_least_square_smoothing_nodelet.cpp "jsk_pcl/MovingLeastSquareSmoothing" "moving_least_square_smoothing")
jsk_pcl_nodelet(src/fisheye_sphere_publisher_nodelet.cpp "jsk_pcl/FisheyeSpherePublisher" "fisheye_sphere_publisher")
jsk_pcl_nodelet(src/plane_supported_cuboid_estimator_nodelet.cpp
  "jsk_pcl/PlaneSupportedCuboidEstimator" "plane_supported_cuboid_estimator")
jsk_pcl_nodelet(src/extract_cuboid_particles_top_n_nodelet.cpp
  "jsk_pcl/ExtractCuboidParticlesTopN" "extract_cuboid_particles_top_n")
jsk_pcl_nodelet(src/interactive_cuboid_likelihood_nodelet.cpp
  "jsk_pcl/InteractiveCuboidLikelihood" "interactive_cuboid_likelihood")
jsk_pcl_nodelet(src/image_rotate_nodelet.cpp
  "jsk_pcl/ImageRotateNodelet" "image_rotate")
jsk_pcl_nodelet(src/octree_change_publisher_nodelet.cpp
  "jsk_pcl/OctreeChangePublisher" "octree_change_publisher")
jsk_pcl_util_nodelet_deprecated(src/tf_transform_cloud_nodelet.cpp
  "jsk_pcl/TfTransformCloud" "tf_transform_cloud")
jsk_pcl_util_nodelet_deprecated(src/tf_transform_bounding_box_nodelet.cpp
  "jsk_pcl/TfTransformBoundingBox" "tf_transform_bounding_box")
jsk_pcl_util_nodelet_deprecated(src/tf_transform_bounding_box_array_nodelet.cpp
  "jsk_pcl/TfTransformBoundingBoxArray" "tf_transform_bounding_box_array")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/RGBColorFilter" "rgb_color_filter")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/HSIColorFilter" "hsi_color_filter")
jsk_pcl_nodelet(src/euclidean_cluster_extraction_nodelet.cpp
  "jsk_pcl/EuclideanClustering" "euclidean_clustering")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposer" "cluster_point_indices_decomposer")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_z_axis_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposerZAxis" "cluster_point_indices_decomposer_z_axis")
jsk_pcl_nodelet(src/resize_points_publisher_nodelet.cpp
  "jsk_pcl/ResizePointsPublisher" "resize_points_publisher")
jsk_pcl_util_nodelet_deprecated(src/normal_concatenater_nodelet.cpp
  "jsk_pcl/NormalConcatenater" "normal_concatenater")
jsk_pcl_nodelet(src/normal_estimation_integral_image_nodelet.cpp
  "jsk_pcl/NormalEstimationIntegralImage" "normal_estimation_integral_image")
jsk_pcl_nodelet(src/region_growing_segmentation_nodelet.cpp
  "jsk_pcl/RegionGrowingSegmentation" "region_growing_segmentation")

jsk_pcl_nodelet(src/organized_multi_plane_segmentation_nodelet.cpp
  "jsk_pcl/OrganizedMultiPlaneSegmentation" "organized_multi_plane_segmentation")
jsk_pcl_nodelet(src/multi_plane_extraction_nodelet.cpp
  "jsk_pcl/MultiPlaneExtraction" "multi_plane_extraction")
jsk_pcl_nodelet(src/selected_cluster_publisher_nodelet.cpp
  "jsk_pcl/SelectedClusterPublisher" "selected_cluster_publisher")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_distance_likelihood_nodelet.cpp
  "jsk_pcl/PolygonArrayDistanceLikelihood" "polygon_array_distance_likelihood")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_area_likelihood_nodelet.cpp
  "jsk_pcl/PolygonArrayAreaLikelihood" "polygon_array_area_likelihood")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_angle_likelihood_nodelet.cpp
  "jsk_pcl/PolygonArrayAngleLikelihood" "polygon_array_angle_likelihood")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_foot_angle_likelihood_nodelet.cpp
  "jsk_pcl/PolygonArrayFootAngleLikelihood" "polygon_array_foot_angle_likelihood")
jsk_pcl_util_nodelet_deprecated(src/pose_with_covariance_stamped_to_gaussian_pointcloud_nodelet.cpp
  "jsk_pcl/PoseWithCovarianceStampedToGaussianPointCloud" "pose_with_covariance_stamped_to_gaussian_pointcloud")
jsk_pcl_util_nodelet_deprecated(src/spherical_pointcloud_simulator_nodelet.cpp
  "jsk_pcl/SphericalPointCloudSimulator" "spherical_pointcloud_simulator")
jsk_pcl_util_nodelet_deprecated(src/polygon_flipper_nodelet.cpp
  "jsk_pcl/PolygonFlipper" "polygon_flipper")
jsk_pcl_util_nodelet_deprecated(src/polygon_points_sampler_nodelet.cpp
  "jsk_pcl/PolygonPointsSampler" "polygon_points_sampler")
jsk_pcl_util_nodelet_deprecated(src/polygon_magnifier_nodelet.cpp
  "jsk_pcl/PolygonMagnifier" "polygon_magnifier")
jsk_pcl_util_nodelet_deprecated(src/planar_pointcloud_simulator_nodelet.cpp
  "jsk_pcl/PlanarPointCloudSimulator" "planar_pointcloud_simulator")
jsk_pcl_util_nodelet_deprecated(src/plane_rejector_nodelet.cpp
  "jsk_pcl/PlaneRejector" "plane_rejector")
jsk_pcl_util_nodelet_deprecated(src/pointcloud_to_cluster_point_indices_nodelet.cpp
  "jsk_pcl/PointCloudToClusterPointIndices" "pointcloud_to_cluster_point_indices")
jsk_pcl_util_nodelet_deprecated(src/static_polygon_array_publisher_nodelet.cpp
  "jsk_pcl/StaticPolygonArrayPublisher" "static_polygon_array_publisher")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_transformer_nodelet.cpp
  "jsk_pcl/PolygonArrayTransformer" "polygon_array_transformer")
jsk_pcl_util_nodelet_deprecated(src/pointcloud_to_stl_nodelet.cpp
  "jsk_pcl/PointCloudToSTL" "pointcloud_to_stl")

if(ml_classifiers_FOUND)
  jsk_pcl_nodelet(src/colorize_segmented_RF_nodelet.cpp
    "jsk_pcl/ColorizeRandomForest" "colorize_random_forest_result")
  jsk_pcl_nodelet(src/colorize_random_points_RF_nodelet.cpp
    "jsk_pcl/ColorizeMapRandomForest" "colorize_random_foreset_result2")
endif(ml_classifiers_FOUND)
jsk_pcl_nodelet(src/environment_plane_modeling_nodelet.cpp
  "jsk_pcl/EnvironmentPlaneModeling" "environment_plane_modeling")
jsk_pcl_nodelet(src/color_histogram_matcher_nodelet.cpp
  "jsk_pcl/ColorHistogramMatcher" "color_histogram_matcher")
jsk_pcl_util_nodelet_deprecated(src/polygon_appender_nodelet.cpp
  "jsk_pcl/PolygonAppender" "polygon_appender")

jsk_pcl_nodelet(src/grid_sampler_nodelet.cpp
  "jsk_pcl/GridSampler" "grid_sampler")
jsk_pcl_nodelet(src/handle_estimator_nodelet.cpp
  "jsk_pcl/HandleEstimator" "handle_estimator")
jsk_pcl_util_nodelet_deprecated(src/delay_pointcloud_nodelet.cpp
  "jsk_pcl/DelayPointCloud" "delay_pointcloud")
jsk_pcl_util_nodelet_deprecated(src/depth_image_error_nodelet.cpp
  "jsk_pcl/DepthImageError" "depth_image_error")
# jsk_pcl_nodelet(src/organize_pointcloud_nodelet.cpp
#   "jsk_pcl/OrganizePointCloud" "organize_pointcloud")
jsk_pcl_nodelet(src/depth_image_creator_nodelet.cpp
  "jsk_pcl/DepthImageCreator" "depth_image_creator")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_wrapper_nodelet.cpp
  "jsk_pcl/PolygonArrayWrapper" "polygon_array_wrapper")
jsk_pcl_util_nodelet_deprecated(src/polygon_array_unwrapper_nodelet.cpp
  "jsk_pcl/PolygonArrayUnwrapper" "polygon_array_unwrapper")
jsk_pcl_nodelet(src/border_estimator_nodelet.cpp
  "jsk_pcl/BorderEstimator" "border_estimator")
jsk_pcl_nodelet(src/region_growing_multiple_plane_segmentation_nodelet.cpp
  "jsk_pcl/RegionGrowingMultiplePlaneSegmentation"
  "region_growing_multiple_plane_segmentation")

IF(${PCL_VERSION} VERSION_GREATER "1.7.2")
  jsk_pcl_nodelet(src/organized_edge_detector_nodelet.cpp
    "jsk_pcl/OrganizedEdgeDetector" "organized_edge_detector")
ENDIF(${PCL_VERSION} VERSION_GREATER "1.7.2")

jsk_pcl_nodelet(src/edge_depth_refinement_nodelet.cpp
  "jsk_pcl/EdgeDepthRefinement" "edge_depth_refinement")
jsk_pcl_nodelet(src/parallel_edge_finder_nodelet.cpp
  "jsk_pcl/ParallelEdgeFinder" "parallel_edge_finder")
jsk_pcl_nodelet(src/edgebased_cube_finder_nodelet.cpp
  "jsk_pcl/EdgebasedCubeFinder" "edgebased_cube_finder")
jsk_pcl_util_nodelet_deprecated(src/colorize_distance_from_plane_nodelet.cpp
  "jsk_pcl/ColorizeDistanceFromPlane" "colorize_distance_from_plane")
jsk_pcl_util_nodelet_deprecated(src/colorize_height_2d_mapping_nodelet.cpp
  "jsk_pcl/ColorizeHeight2DMapping" "colorize_height_2d_mapping")
jsk_pcl_nodelet(src/multi_plane_sac_segmentation_nodelet.cpp
  "jsk_pcl/MultiPlaneSACSegmentation" "multi_plane_sac_segmentation")
jsk_pcl_nodelet(src/bounding_box_filter_nodelet.cpp
  "jsk_pcl/BoundingBoxFilter" "bounding_box_filter")
jsk_pcl_nodelet(src/organized_pass_through_nodelet.cpp
  "jsk_pcl/OrganizedPassThrough" "organized_pass_through")
jsk_pcl_util_nodelet_deprecated(src/plane_reasoner_nodelet.cpp
  "jsk_pcl/PlaneReasoner" "plane_reasoner")
jsk_pcl_nodelet(src/joint_state_static_filter_nodelet.cpp
  "jsk_pcl/JointStateStaticFilter" "joint_state_static_filter")
jsk_pcl_nodelet(src/icp_registration_nodelet.cpp
  "jsk_pcl/ICPRegistration" "icp_registration")
jsk_pcl_util_nodelet_deprecated(src/transform_pointcloud_in_bounding_box_nodelet.cpp
  "jsk_pcl/TransformPointcloudInBoundingBox" "transform_pointcloud_in_bounding_box")
jsk_pcl_nodelet(src/pointcloud_database_server_nodelet.cpp
  "jsk_pcl/PointcloudDatabaseServer" "pointcloud_database_server")
jsk_pcl_nodelet(src/bilateral_filter_nodelet.cpp
  "jsk_pcl/BilateralFilter" "bilateral_filter")
jsk_pcl_nodelet(src/line_segment_detector_nodelet.cpp
  "jsk_pcl/LineSegmentDetector" "line_segment_detector")
jsk_pcl_nodelet(src/line_segment_collector_nodelet.cpp
  "jsk_pcl/LineSegmentCollector" "line_segment_collector")
jsk_pcl_nodelet(src/depth_calibration_nodelet.cpp
  "jsk_pcl/DepthCalibration" "depth_calibration")
jsk_pcl_nodelet(src/tilt_laser_listener_nodelet.cpp
  "jsk_pcl/TiltLaserListener" "tilt_laser_listener")
jsk_pcl_nodelet(src/normal_direction_filter_nodelet.cpp
  "jsk_pcl/NormalDirectionFilter" "normal_direction_filter")
jsk_pcl_nodelet(src/attention_clipper_nodelet.cpp
  "jsk_pcl/AttentionClipper" "attention_clipper")
jsk_pcl_nodelet(src/boundingbox_occlusion_rejector_nodelet.cpp
  "jsk_pcl/BoundingBoxOcclusionRejector" "boundingbox_occlusion_rejector")
jsk_pcl_nodelet(src/roi_clipper_nodelet.cpp
  "jsk_pcl/ROIClipper" "roi_clipper")
jsk_pcl_util_nodelet_deprecated(src/point_indices_to_mask_image_nodelet.cpp
  "jsk_pcl/PointIndicesToMaskImage" "point_indices_to_mask_image")
jsk_pcl_nodelet(src/extract_indices_nodelet.cpp
  "jsk_pcl/ExtractIndices" "extract_indices")
jsk_pcl_util_nodelet_deprecated(src/mask_image_to_depth_considered_mask_image_nodelet.cpp
  "jsk_pcl/MaskImageToDepthConsideredMaskImage" "mask_image_to_depth_considered_mask_image")
jsk_pcl_util_nodelet_deprecated(src/mask_image_to_point_indices_nodelet.cpp
  "jsk_pcl/MaskImageToPointIndices" "mask_image_to_point_indices")
jsk_pcl_nodelet(src/organized_pointcloud_to_point_indices_nodelet.cpp
  "jsk_pcl/OrganizedPointCloudToPointIndices" "organized_pointcloud_to_point_indices")
jsk_pcl_nodelet(src/hinted_handle_estimator_nodelet.cpp
  "jsk_pcl/HintedHandleEstimator" "hinted_handle_estimator")
jsk_pcl_nodelet(src/capture_stereo_synchronizer_nodelet.cpp
  "jsk_pcl/CaptureStereoSynchronizer" "capture_stereo_synchronizer")
jsk_pcl_nodelet(src/linemod_nodelet.cpp
  "jsk_pcl/LINEMODTrainer" "linemod_trainer")
jsk_pcl_nodelet(src/linemod_nodelet.cpp
  "jsk_pcl/LINEMODDetector" "linemod_detector")
jsk_pcl_nodelet(src/intermittent_image_annotator_nodelet.cpp
  "jsk_pcl/IntermittentImageAnnotator" "intermittent_image_annotator")
jsk_pcl_nodelet(src/incremental_model_registration_nodelet.cpp
  "jsk_pcl/IncrementalModelRegistration" "incremental_model_registration")
jsk_pcl_nodelet(src/supervoxel_segmentation_nodelet.cpp
  "jsk_pcl/SupervoxelSegmentation" "supervoxel_segmentation")
jsk_pcl_util_nodelet_deprecated(src/plane_concatenator_nodelet.cpp
  "jsk_pcl/PlaneConcatenator" "plane_concatenator")
jsk_pcl_nodelet(src/add_color_from_image_nodelet.cpp
  "jsk_pcl/AddColorFromImage" "add_color_from_image")
jsk_pcl_nodelet(src/torus_finder_nodelet.cpp "jsk_pcl/TorusFinder" "torus_finder")
jsk_pcl_nodelet(src/mask_image_filter_nodelet.cpp
  "jsk_pcl/MaskImageFilter" "mask_image_filter")
jsk_pcl_nodelet(src/mask_image_cluster_filter_nodelet.cpp
  "jsk_pcl/MaskImageClusterFilter" "mask_image_cluster_filter")
jsk_pcl_util_nodelet_deprecated(src/add_point_indices_nodelet.cpp
  "jsk_pcl/AddPointIndices" "add_point_indices")
jsk_pcl_nodelet(src/find_object_on_plane_nodelet.cpp "jsk_pcl/FindObjectOnPlane" "find_object_on_plane")
jsk_pcl_nodelet(src/hinted_stick_finder_nodelet.cpp
  "jsk_pcl/HintedStickFinder" "hinted_stick_finder")
jsk_pcl_nodelet(src/feature_registration_nodelet.cpp
  "jsk_pcl/FeatureRegistration" "feature_registration")
jsk_pcl_nodelet(src/uniform_sampling_nodelet.cpp
  "jsk_pcl/UniformSampling" "uniform_sampling")
jsk_pcl_nodelet(src/pointcloud_localization_nodelet.cpp
  "jsk_pcl/PointCloudLocalization" "pointcloud_localization")
jsk_pcl_nodelet(src/geometric_consistency_grouping_nodelet.cpp
  "jsk_pcl/GeometricConsistencyGrouping" "geometric_consistency_grouping")
jsk_pcl_nodelet(src/convex_connected_voxels_nodelet.cpp
  "jsk_pcl/ConvexConnectedVoxels" "convex_connected_voxels")
jsk_pcl_nodelet(src/normal_estimation_omp_nodelet.cpp
  "jsk_pcl/NormalEstimationOMP" "normal_estimation_omp")
jsk_pcl_nodelet(src/heightmap_converter_nodelet.cpp
  "jsk_pcl/HeightmapConverter" "heightmap_converter")
jsk_pcl_nodelet(src/heightmap_to_pointcloud_nodelet.cpp
  "jsk_pcl/HeightmapToPointCloud" "heightmap_to_pointcloud")
jsk_pcl_nodelet(src/heightmap_morphological_filtering_nodelet.cpp
  "jsk_pcl/HeightmapMorphologicalFiltering" "heightmap_morphological_filtering")
jsk_pcl_nodelet(src/heightmap_time_accumulation_nodelet.cpp
  "jsk_pcl/HeightmapTimeAccumulation" "heightmap_time_accumulation")
jsk_pcl_nodelet(src/voxel_grid_large_scale_nodelet.cpp
  "jsk_pcl/VoxelGridLargeScale" "voxel_grid_large_scale")
if (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
  jsk_pcl_nodelet(src/kinfu_nodelet.cpp
    "jsk_pcl/Kinfu" "kinfu")
  include_directories(/usr/local/cuda/include)
endif (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
jsk_pcl_util_nodelet_deprecated(src/pcd_reader_with_pose_nodelet.cpp
  "jsk_pcl/PCDReaderWithPose" "pcd_reader_with_pose")
jsk_pcl_nodelet(src/octree_voxel_grid_nodelet.cpp
  "jsk_pcl/OctreeVoxelGrid" "octree_voxel_grid")
# robot_self_filter is released on version greater than indigo
find_package(robot_self_filter QUIET)
if(NOT robot_self_filter_FOUND)
  # check existence of robot_self_filter package because robot_self_filter is not installed by apt on version less than indigo
  pkg_check_modules(robot_self_filter robot_self_filter QUIET)
endif()
if(robot_self_filter_FOUND)
  jsk_pcl_nodelet(src/collision_detector_nodelet.cpp
    "jsk_pcl/CollisionDetector" "collision_detector")
  jsk_pcl_nodelet(src/octomap_server_contact_nodelet.cpp
    "jsk_pcl/OctomapServerContact" "octomap_server_contact")
else(robot_self_filter_FOUND)
  message(AUTHOR_WARNING "robot_self_filter is NOT found. some nodelet depending on robot_self_filter will not be compiled.")
endif(robot_self_filter_FOUND)

add_library(jsk_pcl_ros_base SHARED
  src/region_adjacency_graph.cpp
  src/viewpoint_sampler.cpp)
add_library(jsk_pcl_ros_moveit SHARED src/pointcloud_moveit_filter.cpp)
add_library(jsk_pcl_ros SHARED ${jsk_pcl_nodelet_sources})
if (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
  target_link_libraries(jsk_pcl_ros
    ${PCL_GPU_KINFU_LARGE_SCALE_LIBRARY} ${PCL_GPU_CONTAINERS_LIBRARY}
    ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp
    jsk_pcl_ros_base)
else (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
  target_link_libraries(jsk_pcl_ros
    ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp
    jsk_pcl_ros_base)
endif (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
if (robot_self_filter_FOUND)
  include_directories(include ${robot_self_filter_INCLUDE_DIRS})
  target_link_libraries(jsk_pcl_ros ${robot_self_filter_LIBRARIES})
endif (robot_self_filter_FOUND)

add_dependencies(jsk_pcl_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(jsk_pcl_ros_base ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
add_dependencies(jsk_pcl_ros_moveit ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

target_link_libraries(jsk_pcl_ros_base
  ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
# moveit_ros_perception is excluded from catkin COMPONENTS because of collision about octomap dependency
find_package(moveit_ros_perception)
target_link_libraries(jsk_pcl_ros_moveit
  ${catkin_LIBRARIES} ${moveit_ros_perception_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)


# bench/ directory

add_executable(bench_ransac_plane_estimation bench/bench_ransac_plane_estimation.cpp)
target_link_libraries(bench_ransac_plane_estimation jsk_pcl_ros)

# sample/ directory
add_executable(sample_cube_nearest_point sample/src/sample_cube_nearest_point.cpp)
target_link_libraries(sample_cube_nearest_point jsk_pcl_ros_base)

# tool/ directory
add_executable(range_sensor_error_visualization tool/range_sensor_error_visualization.cpp)
target_link_libraries(range_sensor_error_visualization jsk_pcl_ros)
add_executable(depth_camera_error_visualization tool/depth_camera_error_visualization.cpp)
target_link_libraries(depth_camera_error_visualization jsk_pcl_ros)

generate_messages(DEPENDENCIES
  ${PCL_MSGS} sensor_msgs geometry_msgs jsk_recognition_msgs jsk_footstep_msgs)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message("flags: ${CMAKE_CXX_FLAGS}")

catkin_package(
    DEPENDS pcl
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS} sensor_msgs geometry_msgs jsk_recognition_msgs jsk_recognition_utils
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros jsk_pcl_ros_base jsk_pcl_ros_moveit
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS jsk_pcl_ros jsk_pcl_ros_base jsk_pcl_ros_moveit
  ${jsk_pcl_nodelet_executables}
  ${jsk_pcl_util_nodelet_deprecated_executables}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # install test data by a script
  add_custom_target(install_test_data COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)
  if(TARGET tests)
    add_dependencies(tests install_test_data)
  endif()
  add_rostest_gtest(test_extract_indices test/test_extract_indices.test test/test_extract_indices.cpp)
  target_link_libraries(test_extract_indices ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  # check ros distro >= indigo
  execute_process(
    COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('indigo'[0]))"
    RESULT_VARIABLE DISTRO_GE_INDIGO)
  if(${DISTRO_GE_INDIGO})
    # because of unreleased topic_tools/transform on hydro
    add_rostest(test/test_attention_clipper.test DEPENDENCIES install_test_data)
    add_rostest(test/test_euclidean_segmentation.test DEPENDENCIES install_test_data)
  endif()
endif()
