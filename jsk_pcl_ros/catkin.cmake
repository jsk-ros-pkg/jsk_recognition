# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  # update package.xml, in groovy we need to add pcl to package.xml
  execute_process(COMMAND sed -i s@<run_depend>pcl_ros</run_depend>@<run_depend>pcl_ros</run_depend><run_depend>pcl</run_depend>@g ${PROJECT_SOURCE_DIR}/package.xml)
  execute_process(COMMAND sed -i s@<build_depend>pcl_ros</build_depend>@<build_depend>pcl_ros</build_depend><build_depend>pcl</build_depend>@g ${PROJECT_SOURCE_DIR}/package.xml)
endif($ENV{ROS_DISTRO} STREQUAL "groovy")


# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(PCL_MSGS pcl)
else()
  set(PCL_MSGS pcl_msgs) ## hydro and later
endif()

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(ML_CLASSIFIERS )
else()
  set(ML_CLASSIFIERS ml_classifiers) ## hydro and later
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure pcl_ros nodelet message_generation genmsg
  ${PCL_MSGS} sensor_msgs geometry_msgs
  eigen_conversions tf_conversions tf2_ros tf
  image_transport nodelet cv_bridge
  ${ML_CLASSIFIERS} sklearn jsk_topic_tools)
# only run in hydro
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  find_package(PCL REQUIRED)
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_message_files(FILES PointsArray.msg ClusterPointIndices.msg Int32Stamped.msg SnapItRequest.msg PolygonArray.msg
  ModelCoefficientsArray.msg
  SlicedPointCloud.msg
  BoundingBox.msg
  BoundingBoxArray.msg
  BoundingBoxMovement.msg
  ColorHistogram.msg
  ColorHistogramArray.msg
  SparseOccupancyGridCell.msg
  SparseOccupancyGridColumn.msg
  SparseOccupancyGrid.msg
  SparseOccupancyGridArray.msg
  DepthErrorResult.msg
  ParallelEdge.msg ParallelEdgeArray.msg)
add_service_files(FILES SwitchTopic.srv  TransformScreenpoint.srv CheckCircle.srv RobotPickupReleasePoint.srv  TowerPickUp.srv EuclideanSegment.srv TowerRobotMoveCommand.srv SetPointCloud2.srv
  CallSnapIt.srv CallPolygon.srv
  EnvironmentLock.srv
  PolygonOnEnvironment.srv)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/EuclideanClustering.cfg
  cfg/HSIColorFilter.cfg
  cfg/RGBColorFilter.cfg
  cfg/ImageRotate.cfg
  cfg/RegionGrowingSegmentation.cfg
  cfg/OrganizedMultiPlaneSegmentation.cfg
  cfg/MultiPlaneExtraction.cfg
  cfg/NormalEstimationIntegralImage.cfg
  cfg/PlaneRejector.cfg
  cfg/EnvironmentPlaneModeling.cfg
  cfg/ColorHistogramMatcher.cfg
  cfg/GridSampler.cfg
  cfg/OrganizedEdgeDetector.cfg
  cfg/EdgeDepthRefinement.cfg
  cfg/ParallelEdgeFinder.cfg
  )

find_package(OpenCV REQUIRED core imgproc)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

if(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
else(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_PREFIX}/share/jsk_topic_tools/cmake/nodelet.cmake)
endif(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)

macro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_nodelet_sources)
endmacro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

add_definitions("-O2 -g")

# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
jsk_pcl_nodelet(src/pointcloud_screenpoint_nodelet.cpp "jsk_pcl/PointcloudScreenpoint" "pointcloud_screenpoint")
jsk_pcl_nodelet(src/particle_filter_tracking_nodelet.cpp "jsk_pcl/ParticleFilterTracking" "particle_filter_tracking")
jsk_pcl_nodelet(src/voxel_grid_downsample_manager_nodelet.cpp "jsk_pcl/VoxelGridDownsampleManager" "voxel_grid_downsample_manager")
jsk_pcl_nodelet(src/voxel_grid_downsample_decoder_nodelet.cpp "jsk_pcl/VoxelGridDownsampleDecoder" "voxel_grid_downsample_decoder")
jsk_pcl_nodelet(src/snapit_nodelet.cpp "jsk_pcl/Snapit" "snapit")
jsk_pcl_nodelet(src/keypoints_publisher_nodelet.cpp "jsk_pcl/KeypointsPublisher" "keypoints_publisher")
jsk_pcl_nodelet(src/hinted_plane_detector_nodelet.cpp "jsk_pcl/HintedPlaneDetector" "hinted_plane_detector")
jsk_pcl_nodelet(src/pointcloud_throttle_nodelet.cpp "jsk_pcl/NodeletPointCloudThrottle" "point_cloud_throttle")
jsk_pcl_nodelet(src/centroid_publisher_nodelet.cpp "jsk_pcl/CentroidPublisher" "centroid_publisher")
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
jsk_pcl_nodelet(src/euclidean_cluster_extraction_nodelet.cpp
  "jsk_pcl/EuclideanClustering" "euclidean_clustering")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposer" "cluster_point_indices_decomposer")
jsk_pcl_nodelet(src/cluster_point_indices_decomposer_z_axis_nodelet.cpp
  "jsk_pcl/ClusterPointIndicesDecomposerZAxis" "cluster_point_indices_decomposer_z_axis")
jsk_pcl_nodelet(src/resize_points_publisher_nodelet.cpp
  "jsk_pcl/ResizePointsPublisher" "resize_points_publisher")
jsk_pcl_nodelet(src/normal_concatenater_nodelet.cpp
  "jsk_pcl/NormalConcatenater" "normal_concatenater")
jsk_pcl_nodelet(src/normal_estimation_integral_image_nodelet.cpp
  "jsk_pcl/NormalEstimationIntegralImage" "normal_estimation_integral_image")
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  jsk_pcl_nodelet(src/region_growing_segmentation_nodelet.cpp
    "jsk_pcl/RegionGrowingSegmentation" "region_growing_segmentation")
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")

jsk_pcl_nodelet(src/organized_multi_plane_segmentation_nodelet.cpp
  "jsk_pcl/OrganizedMultiPlaneSegmentation" "organized_multi_plane_segmentation")
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
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  jsk_pcl_nodelet(src/colorize_segmented_RF_nodelet.cpp
    "jsk_pcl/ColorizeRandomForest" "colorize_random_forest_result")
  jsk_pcl_nodelet(src/colorize_random_points_RF_nodelet.cpp
    "jsk_pcl/ColorizeMapRandomForest" "colorize_random_foreset_result2")
endif()
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
jsk_pcl_nodelet(src/depth_image_creator_nodelet.cpp
  "jsk_pcl/DepthImageCreator" "depth_image_creator")
jsk_pcl_nodelet(src/polygon_array_wrapper_nodelet.cpp
  "jsk_pcl/PolygonArrayWrapper" "polygon_array_wrapper")
jsk_pcl_nodelet(src/border_estimator_nodelet.cpp
  "jsk_pcl/BorderEstimator" "border_estimator")

if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  IF(${PCL_VERSION} VERSION_GREATER "1.7.1")
    jsk_pcl_nodelet(src/organized_edge_detector_nodelet.cpp
      "jsk_pcl/OrganizedEdgeDetector" "organized_edge_detector")
  ENDIF(${PCL_VERSION} VERSION_GREATER "1.7.1")
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")

jsk_pcl_nodelet(src/edge_depth_refinement_nodelet.cpp
  "jsk_pcl/EdgeDepthRefinement" "edge_depth_refinement")
jsk_pcl_nodelet(src/parallel_edge_finder_nodelet.cpp
  "jsk_pcl/ParallelEdgeFinder" "parallel_edge_finder")

add_library(jsk_pcl_ros SHARED ${jsk_pcl_nodelet_sources}
  src/grid_index.cpp src/grid_map.cpp src/grid_line.cpp src/geo_util.cpp
  src/pcl_conversion_util.cpp src/pcl_util.cpp)
target_link_libraries(jsk_pcl_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(jsk_pcl_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

generate_messages(DEPENDENCIES ${PCL_MSGS} sensor_msgs geometry_msgs)

catkin_package(
    DEPENDS pcl
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS} sensor_msgs geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS jsk_pcl_ros
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
