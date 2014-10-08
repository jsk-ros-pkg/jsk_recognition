if($ENV{ROS_DISTRO} STREQUAL "groovy")
  file(GLOB _msg_files ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
  foreach(_msg_file ${_msg_files})
    message("convert pcl_msgs -> pcl for groovy ${_msg_file}")
    execute_process(COMMAND grep pcl_msgs ${_msg_file} OUTPUT_VARIABLE _grep_pcl_msgs)
    if (NOT ${_grep_pcl_msgs})
      execute_process(COMMAND sed -i s@^pcl_msgs/@pcl/@g ${_msg_file})
    endif()
  endforeach()
endif()
if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# rosbuild_add_library (jsk_pcl_ros  src/ConvexHullToPlane.cpp)

rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

rosbuild_find_ros_package (pcl_ros)
rosbuild_find_ros_package(jsk_topic_tools)
include(${jsk_topic_tools_PACKAGE_PATH}/cmake/nodelet.cmake)
macro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_nodelet_sources)
endmacro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

jsk_pcl_nodelet(src/pointcloud_screenpoint_nodelet.cpp "jsk_pcl/PointcloudScreenpoint" "pointcloud_screenpoint")
jsk_pcl_nodelet(src/particle_filter_tracking_nodelet.cpp "jsk_pcl/ParticleFilterTracking" "particle_filter_tracking")
jsk_pcl_nodelet(src/voxel_grid_downsample_manager_nodelet.cpp "jsk_pcl/VoxelGridDownsampleManager" "voxel_grid_downsample_manager")
jsk_pcl_nodelet(src/voxel_grid_downsample_decoder_nodelet.cpp "jsk_pcl/VoxelGridDownsampleDecoder" "voxel_grid_downsample_decoder")
jsk_pcl_nodelet(src/snapit_nodelet.cpp "jsk_pcl/Snapit" "snapit")
jsk_pcl_nodelet(src/keypoints_publisher_nodelet.cpp "jsk_pcl/KeypointsPublisher" "keypoints_publisher")
jsk_pcl_nodelet(src/hinted_plane_detector_nodelet.cpp "jsk_pcl/HintedPlaneDetector" "hinted_plane_detector")
jsk_pcl_nodelet(src/resize_points_publisher_nodelet.cpp "jsk_pcl/ResizePointsPublisher" "resize_points_publisher")
jsk_pcl_nodelet(src/depth_image_creator_nodelet.cpp "jsk_pcl/DepthImageCreator" "depth_image_creator")
jsk_pcl_nodelet(src/euclidean_cluster_extraction_nodelet.cpp "jsk_pcl/EuclideanClustering" "euclidean_clustering")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_nodelet.cpp "jsk_pcl/ClusterPointIndicesDecomposer" "cluster_point_indices_decomposer")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_z_axis_nodelet.cpp "jsk_pcl/ClusterPointIndicesDecomposerZAxis" "cluster_point_indices_decomposer_z_axis")
jsk_pcl_nodelet(src/centroid_publisher_nodelet.cpp "jsk_pcl/CentroidPublisher" "centroid_publisher")
jsk_pcl_nodelet(src/pointcloud_throttle_nodelet.cpp
  "jsk_pcl/NodeletPointCloudThrottle" "point_cloud_throttle")
jsk_pcl_nodelet(src/image_throttle_nodelet.cpp
  "jsk_pcl/NodeletImageThrottle" "image_throttle")
jsk_pcl_nodelet(src/image_mux_nodelet.cpp
  "jsk_pcl/NodeletImageMUX" "image_mux")
jsk_pcl_nodelet(src/image_demux_nodelet.cpp
  "jsk_pcl/NodeletImageDEMUX" "image_demux")
jsk_pcl_nodelet(src/image_rotate_nodelet.cpp
  "jsk_pcl/ImageRotateNodelet" "image_rotate")
jsk_pcl_nodelet(src/octree_change_publisher_nodelet.cpp
  "jsk_pcl/OctreeChangePublisher" "octree_change_publisher")
jsk_pcl_nodelet(src/tf_transform_cloud_nodelet.cpp
  "jsk_pcl/TfTransformCloud" "tf_transform_cloud")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/RGBColorFilter" "rgb_color_filter")
jsk_pcl_nodelet(src/color_filter_nodelet.cpp
  "jsk_pcl/HSIColorFilter" "hsi_color_filter")
jsk_pcl_nodelet(src/normal_concatenater_nodelet.cpp
  "jsk_pcl/NormalConcatenater" "normal_concatenater")
jsk_pcl_nodelet(src/normal_estimation_integral_image_nodelet.cpp
  "jsk_pcl/NormalEstimationIntegralImage" "normal_estimation_integral_image")
# PCL1.6 does not support these classes
# jsk_pcl_nodelet(src/region_growing_segmentation_nodelet.cpp
#   "jsk_pcl/RegionGrowingSegmentation" "region_growing_segmentation")
# jsk_pcl_nodelet(src/organized_multi_plane_segmentation_nodelet.cpp
#  "jsk_pcl/OrganizedMultiPlaneSegmentation" "organized_multi_plane_segmentation")
jsk_pcl_nodelet(src/multi_plane_extraction_nodelet.cpp
  "jsk_pcl/MultiPlaneExtraction" "multi_plane_extraction")
jsk_pcl_nodelet(src/selected_cluster_publisher_nodelet.cpp
  "jsk_pcl/SelectedClusterPublisher" "selected_cluster_publisher")
jsk_pcl_nodelet(src/plane_rejector_nodelet.cpp
  "jsk_pcl/PlaneRejector" "plane_rejector")
jsk_pcl_nodelet(src/static_polygon_array_publisher_nodelet.cpp
  "jsk_pcl/StaticPolygonArrayPublisher" "static_polygon_array_publisher")
jsk_pcl_nodelet(src/polygon_array_transformer_nodelet.cpp
  "jsk_pcl/PolygonArrayTransformer" "polygon_array_transformer_nodelet")
jsk_pcl_nodelet(src/environment_plane_modeling_nodelet.cpp
  "jsk_pcl/EnvironmentPlaneModeling" "environment_plane_modeling")
jsk_pcl_nodelet(src/color_histogram_matcher_nodelet.cpp
  "jsk_pcl/ColorHistogramMatcher" "color_histogram_matcher")
jsk_pcl_nodelet(src/polygon_appender_nodelet.cpp
  "jsk_pcl/PolygonAppender" "polygon_appender")
jsk_pcl_nodelet(src/grid_sampler_nodelet.cpp
  "jsk_pcl/GridSampler" "grid_sampler")
jsk_pcl_nodelet(src/handle_estimator_nodelet.cpp
  "jsk_pcl/HandleEstimator" "handle_estimator")
jsk_pcl_nodelet(src/delay_pointcloud_nodelet.cpp
  "jsk_pcl/DelayPointCloud" "delay_pointcloud")
jsk_pcl_nodelet(src/depth_image_error_nodelet.cpp
  "jsk_pcl/DepthImageError" "depth_image_error")
jsk_pcl_nodelet(src/organize_pointcloud_nodelet.cpp
  "jsk_pcl/OrganizePointCloud" "organize_pointcloud")
jsk_pcl_nodelet(src/polygon_array_wrapper_nodelet.cpp
  "jsk_pcl/PolygonArrayWrapper" "polygon_array_wrapper")
jsk_pcl_nodelet(src/border_estimator_nodelet.cpp
  "jsk_pcl/BorderEstimator" "border_estimator")
jsk_pcl_nodelet(src/edge_depth_refinement_nodelet.cpp
  "jsk_pcl/EdgeDepthRefinementr" "edge_depth_refinement")
jsk_pcl_nodelet(src/parallel_edge_finder_nodelet.cpp
  "jsk_pcl/ParallelEdgeFinder" "parallel_edge_finder")
jsk_pcl_nodelet(src/edgebased_cube_finder_nodelet.cpp
  "jsk_pcl/EdgebasedCubeFinder" "edgebased_cube_finder")
jsk_pcl_nodelet(src/colorize_distance_from_plane_nodelet.cpp
  "jsk_pcl/ColorizeDistanceFromPlane" "colorize_distance_from_plane")
jsk_pcl_nodelet(src/multi_plane_sac_segmentation_nodelet.cpp
  "jsk_pcl/MultiPlaneSACSegmentation" "multi_plane_sac_segmentation")
jsk_pcl_nodelet(src/bounding_box_filter_nodelet.cpp
  "jsk_pcl/BoundingBoxFilter" "bounding_box_filter")
jsk_pcl_nodelet(src/organized_pass_through_nodelet.cpp
  "jsk_pcl/OrganizedPassThrough" "organized_pass_through")
jsk_pcl_nodelet(src/plane_reasoner_nodelet.cpp
  "jsk_pcl/PlaneReasoner" "plane_reasoner")
jsk_pcl_nodelet(src/joint_state_static_filter_nodelet.cpp
  "jsk_pcl/JointStateStaticFilter" "joint_state_static_filter")
jsk_pcl_nodelet(src/icp_registration_nodelet.cpp
  "jsk_pcl/ICPRegistration" "icp_registration")
jsk_pcl_nodelet(src/pointcloud_database_server_nodelet.cpp
  "jsk_pcl/PointcloudDatabaseServer" "pointcloud_database_server")
jsk_pcl_nodelet(src/bilateral_filter_nodelet.cpp
  "jsk_pcl/BilateralFilter" "bilateral_filter")

rosbuild_add_library (jsk_pcl_ros
  ${jsk_pcl_nodelet_sources}
  src/grid_index.cpp src/grid_map.cpp src/grid_line.cpp src/geo_util.cpp
  src/pcl_conversion_util.cpp src/pcl_util.cpp
  src/diagnostic_nodelet.cpp
  src/connection_based_nodelet.cpp
)
rosbuild_add_openmp_flags(jsk_pcl_ros)

rosbuild_add_compile_flags (jsk_pcl_ros ${SSE_FLAGS})
rosbuild_link_boost (jsk_pcl_ros system)
#target_link_libraries (jsk_pcl_ros tbb pcl_ros_io pcl_ros_features pcl_ros_filters pcl_ros_segmentation pcl_ros_surface)
target_link_libraries (jsk_pcl_ros tbb pcl_ros_io pcl_ros_filters pcl_ros_tf)

#uncomment if you have defined services
rosbuild_gensrv()

#uncomment if you have defined messages
rosbuild_genmsg()

# roslaunch documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build --nomakefile RESULT_VARIABLE _make_failed)
