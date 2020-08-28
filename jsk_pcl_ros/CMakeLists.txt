# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  eigen_conversions
  # genmsg
  # geometry_msgs
  image_geometry
  image_transport
  interactive_markers
  # jsk_footstep_msgs
  jsk_pcl_ros_utils
  # jsk_recognition_msgs
  jsk_recognition_utils
  # jsk_rviz_plugins # kinfu.h uses jsk_rviz_plugins/OverlayText.h, but jsk_recognition should not depends on jsk_visualization
  jsk_topic_tools
  # kdl_conversions
  kdl_parser
  laser_assembler
  # nodelet
  # octomap_msgs
  octomap_server
  pcl_msgs
  pcl_ros
  # sensor_msgs
  tf
  # tf2_ros
  tf_conversions
  )
find_package(moveit_ros_perception)
find_package(PkgConfig)
pkg_check_modules(ml_classifiers ml_classifiers QUIET)
# only run in hydro
find_package(PCL REQUIRED)
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# For kinfu
find_package(PCL QUIET COMPONENTS gpu_kinfu_large_scale)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
if(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
endif()

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
  cfg/PeopleDetection.cfg
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
  cfg/ColorBasedRegionGrowingSegmentation.cfg
  cfg/OrganizedMultiPlaneSegmentation.cfg
  cfg/MultiPlaneExtraction.cfg
  cfg/NormalEstimationIntegralImage.cfg
  cfg/EnvironmentPlaneModeling.cfg
  cfg/ColorHistogram.cfg
  cfg/ColorHistogramClassifier.cfg
  cfg/ColorHistogramFilter.cfg
  cfg/ColorHistogramMatcher.cfg
  cfg/ColorHistogramVisualizer.cfg
  cfg/ClusterPointIndicesDecomposer.cfg
  cfg/GridSampler.cfg
  cfg/OrganizedEdgeDetector.cfg
  cfg/EdgeDepthRefinement.cfg
  cfg/ParallelEdgeFinder.cfg
  cfg/EdgebasedCubeFinder.cfg
  cfg/MultiPlaneSACSegmentation.cfg
  cfg/BoundingBoxFilter.cfg
  cfg/ResizePointsPublisher.cfg
  cfg/RearrangeBoundingBox.cfg
  cfg/LINEMODDetector.cfg
  cfg/SupervoxelSegmentation.cfg
  cfg/FeatureRegistration.cfg
  cfg/FisheyeSphere.cfg
  cfg/MovingLeastSquareSmoothing.cfg
  cfg/OctreeVoxelGrid.cfg
  cfg/OctreeChangePublisher.cfg
  cfg/ExtractTopPolygonLikelihood.cfg
  cfg/PointcloudDatabaseServer.cfg
  cfg/TargetAdaptiveTracking.cfg
  cfg/PPFRegistration.cfg
  cfg/PrimitiveShapeClassifier.cfg
  cfg/Kinfu.cfg
  cfg/PointcloudScreenpoint.cfg
  )

find_package(OpenCV REQUIRED core imgproc)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()
macro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_nodelet_sources jsk_pcl_nodelet_executables)
endmacro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

macro(jsk_pcl_nodelet_upstream _upstream _nodelet_class _single_nodelet_exec_name)
  message("DEPRECATION WARNING: nodelet '${_nodelet_class}' is moved to upstream package '${_upstream}'.")
  jsk_nodelet(dummy.cpp ${_nodelet_class} ${_single_nodelet_exec_name} jsk_pcl_nodelet_dummy_sources jsk_pcl_nodelet_executables)
endmacro(jsk_pcl_nodelet_upstream _upstream _nodelet_class _single_nodelet_exec_name)

if ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O0")
else ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O2 -g")
endif ($ENV{TRAVIS_JOB_ID})

# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
jsk_pcl_nodelet(src/pointcloud_screenpoint_nodelet.cpp "jsk_pcl/PointcloudScreenpoint" "pointcloud_screenpoint")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/NormalFlipToFrame" "normal_flip_to_frame")
if(OPENMP_FOUND)
  jsk_pcl_nodelet(src/particle_filter_tracking_nodelet.cpp "jsk_pcl/ParticleFilterTracking" "particle_filter_tracking")
endif()
jsk_pcl_nodelet(src/fuse_images.cpp "jsk_pcl/FuseRGBImages" "fuse_rgb_images")
jsk_pcl_nodelet(src/fuse_images.cpp "jsk_pcl/FuseDepthImages" "fuse_depth_images")
jsk_pcl_nodelet(src/voxel_grid_downsample_manager_nodelet.cpp "jsk_pcl/VoxelGridDownsampleManager" "voxel_grid_downsample_manager")
jsk_pcl_nodelet(src/voxel_grid_downsample_decoder_nodelet.cpp "jsk_pcl/VoxelGridDownsampleDecoder" "voxel_grid_downsample_decoder")
jsk_pcl_nodelet(src/snapit_nodelet.cpp "jsk_pcl/Snapit" "snapit")
jsk_pcl_nodelet(src/keypoints_publisher_nodelet.cpp "jsk_pcl/KeypointsPublisher" "keypoints_publisher")
jsk_pcl_nodelet(src/hinted_plane_detector_nodelet.cpp "jsk_pcl/HintedPlaneDetector" "hinted_plane_detector")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/CentroidPublisher" "centroid_publisher")
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
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/TfTransformCloud" "tf_transform_cloud")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/TfTransformBoundingBox" "tf_transform_bounding_box")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/TfTransformBoundingBoxArray" "tf_transform_bounding_box_array")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/RGBColorFilter" "rgb_color_filter")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/HSIColorFilter" "hsi_color_filter")
jsk_pcl_nodelet(src/euclidean_cluster_extraction_nodelet.cpp
  "jsk_pcl/EuclideanClustering" "euclidean_clustering")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposer" "cluster_point_indices_decomposer")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposerZAxis" "cluster_point_indices_decomposer_z_axis")
jsk_pcl_nodelet(src/resize_points_publisher_nodelet.cpp
  "jsk_pcl/ResizePointsPublisher" "resize_points_publisher")
jsk_pcl_nodelet(src/rearrange_bounding_box_nodelet.cpp
  "jsk_pcl/RearrangeBoundingBox" "rearrange_bounding_box")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/NormalConcatenater" "normal_concatenater")
jsk_pcl_nodelet(src/normal_estimation_integral_image_nodelet.cpp
  "jsk_pcl/NormalEstimationIntegralImage" "normal_estimation_integral_image")
jsk_pcl_nodelet(src/region_growing_segmentation_nodelet.cpp
  "jsk_pcl/RegionGrowingSegmentation" "region_growing_segmentation")
jsk_pcl_nodelet(src/color_based_region_growing_segmentation_nodelet.cpp
  "jsk_pcl/ColorBasedRegionGrowingSegmentation" "color_based_region_growing_segmentation")
jsk_pcl_nodelet(src/organized_multi_plane_segmentation_nodelet.cpp
  "jsk_pcl/OrganizedMultiPlaneSegmentation" "organized_multi_plane_segmentation")
jsk_pcl_nodelet(src/multi_plane_extraction_nodelet.cpp
  "jsk_pcl/MultiPlaneExtraction" "multi_plane_extraction")
jsk_pcl_nodelet(src/selected_cluster_publisher_nodelet.cpp
  "jsk_pcl/SelectedClusterPublisher" "selected_cluster_publisher")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayDistanceLikelihood" "polygon_array_distance_likelihood")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayAreaLikelihood" "polygon_array_area_likelihood")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayAngleLikelihood" "polygon_array_angle_likelihood")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayFootAngleLikelihood" "polygon_array_foot_angle_likelihood")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PoseWithCovarianceStampedToGaussianPointCloud" "pose_with_covariance_stamped_to_gaussian_pointcloud")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/SphericalPointCloudSimulator" "spherical_pointcloud_simulator")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonFlipper" "polygon_flipper")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonPointsSampler" "polygon_points_sampler")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonMagnifier" "polygon_magnifier")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PlanarPointCloudSimulator" "planar_pointcloud_simulator")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PlaneRejector" "plane_rejector")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PointCloudToClusterPointIndices" "pointcloud_to_cluster_point_indices")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/StaticPolygonArrayPublisher" "static_polygon_array_publisher")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayTransformer" "polygon_array_transformer")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PointCloudToSTL" "pointcloud_to_stl")

if(ml_classifiers_FOUND)
  jsk_pcl_nodelet(src/colorize_segmented_RF_nodelet.cpp
    "jsk_pcl/ColorizeRandomForest" "colorize_random_forest")
  jsk_pcl_nodelet(src/colorize_random_points_RF_nodelet.cpp
    "jsk_pcl/ColorizeMapRandomForest" "colorize_map_random_forest")
endif(ml_classifiers_FOUND)
jsk_pcl_nodelet(src/environment_plane_modeling_nodelet.cpp
  "jsk_pcl/EnvironmentPlaneModeling" "environment_plane_modeling")
if(jsk_topic_tools_VERSION VERSION_LESS 2.2.4)
  message(WARNING "Building some nodelets are skipped if jsk_topic_tools < 2.2.4")
else()
  jsk_pcl_nodelet(src/color_histogram_nodelet.cpp "jsk_pcl/ColorHistogram" "color_histogram")
  jsk_pcl_nodelet(src/color_histogram_classifier_nodelet.cpp "jsk_pcl/ColorHistogramClassifier" "color_histogram_classifier")
  jsk_pcl_nodelet(src/color_histogram_filter_nodelet.cpp "jsk_pcl/ColorHistogramFilter" "color_histogram_filter")
  jsk_pcl_nodelet(src/primitive_shape_classifier_nodelet.cpp
    "jsk_pcl/PrimitiveShapeClassifier" "primitive_shape_classifier")
endif()
jsk_pcl_nodelet(src/color_histogram_matcher_nodelet.cpp
  "jsk_pcl/ColorHistogramMatcher" "color_histogram_matcher")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonAppender" "polygon_appender")

jsk_pcl_nodelet(src/grid_sampler_nodelet.cpp
  "jsk_pcl/GridSampler" "grid_sampler")
jsk_pcl_nodelet(src/handle_estimator_nodelet.cpp
  "jsk_pcl/HandleEstimator" "handle_estimator")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/DelayPointCloud" "delay_pointcloud")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/DepthImageError" "depth_image_error")
# jsk_pcl_nodelet(src/organize_pointcloud_nodelet.cpp
#   "jsk_pcl/OrganizePointCloud" "organize_pointcloud")
jsk_pcl_nodelet(src/depth_image_creator_nodelet.cpp
  "jsk_pcl/DepthImageCreator" "depth_image_creator")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayWrapper" "polygon_array_wrapper")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PolygonArrayUnwrapper" "polygon_array_unwrapper")
jsk_pcl_nodelet(src/border_estimator_nodelet.cpp
  "jsk_pcl/BorderEstimator" "border_estimator")
jsk_pcl_nodelet(src/region_growing_multiple_plane_segmentation_nodelet.cpp
  "jsk_pcl/RegionGrowingMultiplePlaneSegmentation"
  "region_growing_multiple_plane_segmentation")

if(${PCL_VERSION} VERSION_GREATER "1.7.2")
  jsk_pcl_nodelet(src/organized_edge_detector_nodelet.cpp
    "jsk_pcl/OrganizedEdgeDetector" "organized_edge_detector")
endif(${PCL_VERSION} VERSION_GREATER "1.7.2")

jsk_pcl_nodelet(src/edge_depth_refinement_nodelet.cpp
  "jsk_pcl/EdgeDepthRefinement" "edge_depth_refinement")
jsk_pcl_nodelet(src/parallel_edge_finder_nodelet.cpp
  "jsk_pcl/ParallelEdgeFinder" "parallel_edge_finder")
jsk_pcl_nodelet(src/edgebased_cube_finder_nodelet.cpp
  "jsk_pcl/EdgebasedCubeFinder" "edgebased_cube_finder")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/ColorizeDistanceFromPlane" "colorize_distance_from_plane")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/ColorizeHeight2DMapping" "colorize_height_2d_mapping")
jsk_pcl_nodelet(src/multi_plane_sac_segmentation_nodelet.cpp
  "jsk_pcl/MultiPlaneSACSegmentation" "multi_plane_sac_segmentation")
jsk_pcl_nodelet(src/bounding_box_filter_nodelet.cpp
  "jsk_pcl/BoundingBoxFilter" "bounding_box_filter")
jsk_pcl_nodelet(src/organized_pass_through_nodelet.cpp
  "jsk_pcl/OrganizedPassThrough" "organized_pass_through")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PlaneReasoner" "plane_reasoner")
jsk_pcl_nodelet(src/joint_state_static_filter_nodelet.cpp
  "jsk_pcl/JointStateStaticFilter" "joint_state_static_filter")
jsk_pcl_nodelet(src/icp_registration_nodelet.cpp
  "jsk_pcl/ICPRegistration" "icp_registration")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/TransformPointcloudInBoundingBox" "transform_pointcloud_in_bounding_box")
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
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PointIndicesToMaskImage" "point_indices_to_mask_image")
jsk_pcl_nodelet(src/extract_indices_nodelet.cpp
  "jsk_pcl/ExtractIndices" "extract_indices")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/MaskImageToDepthConsideredMaskImage" "mask_image_to_depth_considered_mask_image")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/MaskImageToPointIndices" "mask_image_to_point_indices")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/OrganizedPointCloudToPointIndices" "organized_pointcloud_to_point_indices")
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
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PlaneConcatenator" "plane_concatenator")
jsk_pcl_nodelet(src/add_color_from_image_nodelet.cpp
  "jsk_pcl/AddColorFromImage" "add_color_from_image")
jsk_pcl_nodelet(src/add_color_from_image_to_organized_nodelet.cpp
  "jsk_pcl/AddColorFromImageToOrganized" "add_color_from_image_to_organized")
jsk_pcl_nodelet(src/torus_finder_nodelet.cpp "jsk_pcl/TorusFinder" "torus_finder")
jsk_pcl_nodelet(src/mask_image_filter_nodelet.cpp
  "jsk_pcl/MaskImageFilter" "mask_image_filter")
jsk_pcl_nodelet(src/mask_image_cluster_filter_nodelet.cpp
  "jsk_pcl/MaskImageClusterFilter" "mask_image_cluster_filter")
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/AddPointIndices" "add_point_indices")
jsk_pcl_nodelet(src/find_object_on_plane_nodelet.cpp "jsk_pcl/FindObjectOnPlane" "find_object_on_plane")
jsk_pcl_nodelet(src/hinted_stick_finder_nodelet.cpp
  "jsk_pcl/HintedStickFinder" "hinted_stick_finder")
jsk_pcl_nodelet(src/feature_registration_nodelet.cpp
  "jsk_pcl/FeatureRegistration" "feature_registration")
jsk_pcl_nodelet(src/uniform_sampling_nodelet.cpp
  "jsk_pcl/UniformSampling" "uniform_sampling")
jsk_pcl_nodelet(src/pointcloud_localization_nodelet.cpp
  "jsk_pcl/PointCloudLocalization" "pointcloud_localization")
jsk_pcl_nodelet(src/people_detection_nodelet.cpp
  "jsk_pcl/PeopleDetection" "people_detection")
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
jsk_pcl_nodelet(src/ppf_registration_nodelet.cpp
  "jsk_pcl/PPFRegistration" "ppf_registration")

# kinfu.h uses jsk_rviz_plugins/OverlayText.h, but jsk_recognition should not depends on jsk_visualization
find_package(jsk_rviz_plugins QUIET)
if (PCL_GPU_KINFU_LARGE_SCALE_FOUND AND jsk_rviz_plugins_FOUND)
  include_directories(${jsk_rviz_plugins_INCLUDE_DIRS})
  jsk_pcl_nodelet(src/kinfu_nodelet.cpp
    "jsk_pcl/Kinfu" "kinfu")
  include_directories(/usr/local/cuda/include)
endif (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
jsk_pcl_nodelet_upstream(jsk_pcl_ros_utils "jsk_pcl/PCDReaderWithPose" "pcd_reader_with_pose")
jsk_pcl_nodelet(src/target_adaptive_tracking_nodelet.cpp
  "jsk_pcl/TargetAdaptiveTracking" "target_adaptive_tracking")
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
add_library(jsk_pcl_ros SHARED ${jsk_pcl_nodelet_sources})
if (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
  target_link_libraries(jsk_pcl_ros
    ${PCL_GPU_KINFU_LARGE_SCALE_LIBRARY} ${PCL_GPU_CONTAINERS_LIBRARY}
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp
    jsk_pcl_ros_base)
else (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
  target_link_libraries(jsk_pcl_ros
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp
    jsk_pcl_ros_base)
endif (PCL_GPU_KINFU_LARGE_SCALE_FOUND)
if (robot_self_filter_FOUND)
  include_directories(include ${robot_self_filter_INCLUDE_DIRS})
  target_link_libraries(jsk_pcl_ros ${robot_self_filter_LIBRARIES})
endif (robot_self_filter_FOUND)

add_dependencies(jsk_pcl_ros ${PROJECT_NAME}_gencfg)
add_dependencies(jsk_pcl_ros_base ${PROJECT_NAME}_gencfg)

target_link_libraries(jsk_pcl_ros_base
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

set(jsk_pcl_ros_install_libraries jsk_pcl_ros jsk_pcl_ros_base)
if(moveit_ros_perception_FOUND)
  add_library(jsk_pcl_ros_moveit SHARED src/pointcloud_moveit_filter.cpp)
  add_dependencies(jsk_pcl_ros_moveit ${PROJECT_NAME}_gencfg)
  target_link_libraries(jsk_pcl_ros_moveit
    ${catkin_LIBRARIES} ${moveit_ros_perception_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
  list(APPEND jsk_pcl_ros_install_libraries jsk_pcl_ros_moveit)
else(moveit_ros_perception_FOUND)
  message(STATUS "moveit_ros_perception is not found, so skip creating jsk_pcl_ros_moveit")
endif(moveit_ros_perception_FOUND)

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

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message("flags: ${CMAKE_CXX_FLAGS}")

catkin_package(
    DEPENDS PCL
    CATKIN_DEPENDS pcl_ros ${PCL_MSGS} jsk_recognition_utils
    INCLUDE_DIRS include
    LIBRARIES ${jsk_pcl_ros_install_libraries}
)

# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${jsk_pcl_ros_install_libraries}
  ${jsk_pcl_nodelet_executables}
  bench_ransac_plane_estimation
  depth_camera_error_visualization
  range_sensor_error_visualization
  sample_cube_nearest_point
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config euslisp launch plugins sample scripts test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  find_package(jsk_perception REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_extract_indices test/test_extract_indices.test test/test_extract_indices.cpp)
  target_link_libraries(test_extract_indices ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # because of unreleased topic_tools/transform on hydro
    add_rostest(test/test_add_color_from_image.test)
    add_rostest(test/test_add_color_from_image_to_organized.test)
    add_rostest(test/test_bilateral_filter.test)
    add_rostest(test/test_border_estimator.test)
    add_rostest(test/test_boundingbox_occlusion_rejector.test)
    add_rostest(test/test_calculate_polygon_from_imu.test)
    add_rostest(test/test_capture_stereo_synchronizer.test)
    add_rostest(test/test_collision_detector.test)
    if(ml_classifiers_FOUND)
      # Wait for upstream: ml_classifiers is not yet released in jade, kinetic, ...
      # (https://github.com/jsk-ros-pkg/jsk_recognition/pull/2385#issuecomment-451349430)
      add_rostest(test/test_colorize_map_random_forest.test)
      add_rostest(test/test_colorize_random_forest.test)
    endif()
    add_rostest(test/test_depth_calibration.test)
    add_rostest(test/test_depth_error_calibration.test)
    add_rostest(test/test_display_bounding_box_array.test)
    add_rostest(test/test_dump_depth_error.test)
    add_rostest(test/test_octree_voxel_grid.test)
    add_rostest(test/test_octomap_contact.test)
    add_rostest(test/test_handle_estimator.test)
    add_rostest(test/test_hsi_color_filter.test)
    add_rostest(test/test_icp_registration.test)
    add_rostest(test/test_attention_clipper.test)
    add_rostest(test/test_cluster_point_indices_decomposer.test)
    add_rostest(test/test_cluster_point_indices_decomposer_bbox.test)
    add_rostest(test/test_cluster_point_indices_decomposer_sort_by.test)
    add_rostest(test/test_depth_image_creator.test)
    if(${PCL_VERSION} VERSION_GREATER "1.7.2")
      # because of jsk_pcl/OrganizedEdgeDetector
      add_rostest(test/test_edge_depth_refinement.test)
    endif()
    add_rostest(test/test_edgebased_cube_finder.test)
    add_rostest(test/test_environment_plane_modeling.test)
    add_rostest(test/test_euclidean_segmentation.test)
    add_rostest(test/test_extract_cuboid_particles_top_n.test)
    add_rostest(test/test_extract_top_polygon_likelihood.test)
    add_rostest(test/test_feature_registration.test)
    add_rostest(test/test_find_object_on_plane.test)
    add_rostest(test/test_fisheye_sphere_publisher.test)
    add_rostest(test/test_fuse_depth_images.test)
    add_rostest(test/test_fuse_rgb_images.test)
    add_rostest(test/test_grid_sampler.test)
    add_rostest(test/test_heightmap_converter.test)
    add_rostest(test/test_heightmap_morphological_filtering.test)
    add_rostest(test/test_heightmap_time_accumulation.test)
    add_rostest(test/test_heightmap_to_pointcloud.test)
    add_rostest(test/test_hinted_handle_estimator.test)
    add_rostest(test/test_hinted_plane_detector.test)
    add_rostest(test/test_hinted_stick_finder.test)
    # TODO(iory): https://github.com/jsk-ros-pkg/jsk_recognition/issues/2007
    # add_rostest(test/test_people_detection.test DEPENDENCIES install_test_data)
    add_rostest(test/test_in_hand_recognition_manager.test)
    add_rostest(test/test_incremental_model_registration.test)
    add_rostest(test/test_interactive_cuboid_likelihood.test)
    add_rostest(test/test_intermittent_image_annotator.test)
    add_rostest(test/test_joint_state_static_filter.test)
    add_rostest(test/test_keypoints_publisher.test)
    if(PCL_GPU_KINFU_LARGE_SCALE_FOUND)
      add_rostest(test/test_kinfu.test)
    endif()
    add_rostest(test/test_line_segment_collector.test)
    add_rostest(test/test_line_segment_detector.test)
    add_rostest(test/test_linemod_detector.test)
    add_rostest(test/test_linemod_trainer.test)
    add_rostest(test/test_marker_appender.test)
    add_rostest(test/test_mask_image_cluster_filter.test)
    add_rostest(test/test_mask_image_filter.test)
    add_rostest(test/test_moving_least_square_smoothing.test)
    add_rostest(test/test_multi_euclidean_clustering.test)
    add_rostest(test/test_multi_plane_extraction.test)
    add_rostest(test/test_multi_plane_sac_segmentation.test)
    add_rostest(test/test_normal_direction_filter.test)
    add_rostest(test/test_normal_estimation_integral_image.test)
    add_rostest(test/test_normal_estimation_omp.test)
    add_rostest(test/test_octree_change_publisher.test)
    if(${PCL_VERSION} VERSION_GREATER "1.7.2")
      add_rostest(test/test_organized_edge_detector.test)
    endif()
    add_rostest(test/test_organized_multi_plane_segmentation.test)
    add_rostest(test/test_organized_pass_through.test)
    add_rostest(test/test_parallel_edge_finder.test)
    add_rostest(test/test_particle_filter_tracking.test)
    add_rostest(test/test_particle_filter_tracking_change_detection.test)
    add_rostest(test/test_particle_filter_tracking_service_renew.test)
    add_rostest(test/test_plane_supported_cuboid_estimator.test)
    add_rostest(test/test_plane_time_ensync_for_recognition.test)
    add_rostest(test/test_pointcloud_database_server.test)
    add_rostest(test/test_pointcloud_localization.test)
    add_rostest(test/test_pointcloud_screenpoint.test)
    add_rostest(test/test_publish_clicked_point_bbox.test)
    add_rostest(test/test_convex_connected_voxels.test)
    add_rostest(test/test_organized_pointcloud_to_point_indices.test)
    add_rostest(test/test_rearrange_bounding_box.test)
    add_rostest(test/test_region_growing_multiple_plane_segmentation.test)
    add_rostest(test/test_region_growing_segmentation.test)
    add_rostest(test/test_resize_points_publisher.test)
    add_rostest(test/test_rgb_color_filter.test)
    add_rostest(test/test_roi_clipper.test)
    add_rostest(test/test_selected_cluster_publisher.test)
    add_rostest(test/test_snapit.test)
    add_rostest(test/test_supervoxel_segmentation.test)
    add_rostest(test/test_target_adaptive_tracking.test)
    add_rostest(test/test_tilt_laser_listener.test)
    add_rostest(test/test_torus_finder.test)
    add_rostest(test/test_color_based_region_growing_segmentation.test)
    add_rostest(test/test_detect_graspable_poses_pcabase.test)
    if(jsk_topic_tools_VERSION VERSION_GREATER 2.2.4)
      add_rostest(test/color_histogram.test)
      add_rostest(test/primitive_shape_classifier.test)
    endif()
    add_rostest(test/test_voxel_grid_downsample.test)
    add_rostest(test/test_voxel_grid_large_scale.test)
  endif()
  roslaunch_add_file_check(launch/openni_local.launch)
  roslaunch_add_file_check(launch/openni2_local.launch)
  roslaunch_add_file_check(launch/openni_remote.launch)
  roslaunch_add_file_check(launch/openni2_remote.launch)
endif()
