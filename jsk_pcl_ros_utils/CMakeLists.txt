# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros_utils)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

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
  jsk_recognition_utils
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
pkg_check_modules(${ML_CLASSIFIERS} ml_classifiers QUIET)
# only run in hydro
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  find_package(PCL REQUIRED)
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
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

# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py --pkg-path ${PROJECT_SOURCE_DIR})

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/BoundingBoxArrayToBoundingBox.cfg
  cfg/CloudOnPlane.cfg
  cfg/ClusterPointIndicesToPointIndices.cfg
  cfg/ColorizeDistanceFromPlane.cfg
  cfg/DelayPointCloud.cfg
  cfg/MaskImageToDepthConsideredMaskImage.cfg
  cfg/PlanarPointCloudSimulator.cfg
  cfg/PlaneConcatenator.cfg
  cfg/PlaneReasoner.cfg
  cfg/PlaneRejector.cfg
  cfg/PointCloudToMaskImage.cfg
  cfg/PointCloudToPCD.cfg
  cfg/PolygonArrayAreaLikelihood.cfg
  cfg/PolygonArrayLikelihoodFilter.cfg
  cfg/PolygonArrayUnwrapper.cfg
  cfg/PolygonMagnifier.cfg
  cfg/PolygonPointsSampler.cfg
  cfg/PoseWithCovarianceStampedToGaussianPointCloud.cfg
  cfg/SphericalPointCloudSimulator.cfg
)

find_package(OpenCV REQUIRED core imgproc)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

macro(jsk_pcl_util_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_pcl_util_nodelet_sources jsk_pcl_util_nodelet_executables)
endmacro(jsk_pcl_util_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

if ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O0")
else ($ENV{TRAVIS_JOB_ID})
  add_definitions("-O2 -g")
endif ($ENV{TRAVIS_JOB_ID})

jsk_pcl_util_nodelet(src/bounding_box_array_to_bounding_box_nodelet.cpp "jsk_pcl_utils/BoundingBoxArrayToBoundingBox"
  "bounding_box_array_to_bounding_box")
jsk_pcl_util_nodelet(src/normal_flip_to_frame_nodelet.cpp "jsk_pcl_utils/NormalFlipToFrame"
  "normal_flip_to_frame")
jsk_pcl_util_nodelet(src/centroid_publisher_nodelet.cpp "jsk_pcl_utils/CentroidPublisher" "centroid_publisher")
jsk_pcl_util_nodelet(src/cloud_on_plane_nodelet.cpp "jsk_pcl/CloudOnPlane"
  "cloud_on_plane")
jsk_pcl_util_nodelet(src/tf_transform_cloud_nodelet.cpp
  "jsk_pcl_utils/TfTransformCloud" "tf_transform_cloud")
jsk_pcl_util_nodelet(src/tf_transform_bounding_box_nodelet.cpp
  "jsk_pcl_utils/TfTransformBoundingBox" "tf_transform_bounding_box")
jsk_pcl_util_nodelet(src/tf_transform_bounding_box_array_nodelet.cpp
  "jsk_pcl_utils/TfTransformBoundingBoxArray" "tf_transform_bounding_box_array")
jsk_pcl_util_nodelet(src/normal_concatenater_nodelet.cpp
  "jsk_pcl_utils/NormalConcatenater" "normal_concatenater")
jsk_pcl_util_nodelet(src/polygon_array_distance_likelihood_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayDistanceLikelihood" "polygon_array_distance_likelihood")
jsk_pcl_util_nodelet(src/polygon_array_area_likelihood_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayAreaLikelihood" "polygon_array_area_likelihood")
jsk_pcl_util_nodelet(src/polygon_array_angle_likelihood_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayAngleLikelihood" "polygon_array_angle_likelihood")
jsk_pcl_util_nodelet(src/polygon_array_foot_angle_likelihood_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayFootAngleLikelihood" "polygon_array_foot_angle_likelihood")
jsk_pcl_util_nodelet(src/polygon_array_likelihood_filter_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayLikelihoodFilter" "polygon_array_likelihood_filter")
jsk_pcl_util_nodelet(src/pose_with_covariance_stamped_to_gaussian_pointcloud_nodelet.cpp
  "jsk_pcl_utils/PoseWithCovarianceStampedToGaussianPointCloud" "pose_with_covariance_stamped_to_gaussian_pointcloud")
jsk_pcl_util_nodelet(src/spherical_pointcloud_simulator_nodelet.cpp
  "jsk_pcl_utils/SphericalPointCloudSimulator" "spherical_pointcloud_simulator")
jsk_pcl_util_nodelet(src/polygon_flipper_nodelet.cpp
  "jsk_pcl_utils/PolygonFlipper" "polygon_flipper")
jsk_pcl_util_nodelet(src/polygon_points_sampler_nodelet.cpp
  "jsk_pcl_utils/PolygonPointsSampler" "polygon_points_sampler")
jsk_pcl_util_nodelet(src/polygon_magnifier_nodelet.cpp
  "jsk_pcl_utils/PolygonMagnifier" "polygon_magnifier")
jsk_pcl_util_nodelet(src/planar_pointcloud_simulator_nodelet.cpp
  "jsk_pcl_utils/PlanarPointCloudSimulator" "planar_pointcloud_simulator")
jsk_pcl_util_nodelet(src/plane_rejector_nodelet.cpp
  "jsk_pcl_utils/PlaneRejector" "plane_rejector")
if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
  jsk_pcl_util_nodelet(src/cluster_point_indices_to_point_indices_nodelet.cpp
    "jsk_pcl_utils/ClusterPointIndicesToPointIndices" "cluster_point_indices_to_point_indices")
endif()
jsk_pcl_util_nodelet(src/pointcloud_xyz_to_xyzrgb_nodelet.cpp
  "jsk_pcl_utils/PointCloudXYZToXYZRGB" "pointcloud_xyz_to_xyzrgb")
jsk_pcl_util_nodelet(src/pointcloud_xyzrgb_to_xyz_nodelet.cpp
  "jsk_pcl_utils/PointCloudXYZRGBToXYZ" "pointcloud_xyzrgb_to_xyz")
jsk_pcl_util_nodelet(src/pointcloud_to_cluster_point_indices_nodelet.cpp
  "jsk_pcl_utils/PointCloudToClusterPointIndices" "pointcloud_to_cluster_point_indices")
jsk_pcl_util_nodelet(src/pointcloud_to_mask_image_nodelet.cpp
  "jsk_pcl_utils/PointCloudToMaskImage" "pointcloud_to_mask_image")
jsk_pcl_util_nodelet(src/static_polygon_array_publisher_nodelet.cpp
  "jsk_pcl_utils/StaticPolygonArrayPublisher" "static_polygon_array_publisher")
jsk_pcl_util_nodelet(src/polygon_array_transformer_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayTransformer" "polygon_array_transformer")
jsk_pcl_util_nodelet(src/pointcloud_to_stl_nodelet.cpp
  "jsk_pcl_utils/PointCloudToSTL" "pointcloud_to_stl")
jsk_pcl_util_nodelet(src/pointcloud_to_point_indices_nodelet.cpp
  "jsk_pcl_utils/PointCloudToPointIndices" "pointcloud_to_point_indices")
jsk_pcl_util_nodelet(src/polygon_appender_nodelet.cpp
  "jsk_pcl_utils/PolygonAppender" "polygon_appender")
jsk_pcl_util_nodelet(src/delay_pointcloud_nodelet.cpp
  "jsk_pcl_utils/DelayPointCloud" "delay_pointcloud")
jsk_pcl_util_nodelet(src/depth_image_error_nodelet.cpp
  "jsk_pcl_utils/DepthImageError" "depth_image_error")
jsk_pcl_util_nodelet(src/polygon_array_wrapper_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayWrapper" "polygon_array_wrapper")
jsk_pcl_util_nodelet(src/polygon_array_unwrapper_nodelet.cpp
  "jsk_pcl_utils/PolygonArrayUnwrapper" "polygon_array_unwrapper")
jsk_pcl_util_nodelet(src/colorize_distance_from_plane_nodelet.cpp
  "jsk_pcl_utils/ColorizeDistanceFromPlane" "colorize_distance_from_plane")
jsk_pcl_util_nodelet(src/colorize_height_2d_mapping_nodelet.cpp
  "jsk_pcl_utils/ColorizeHeight2DMapping" "colorize_height_2d_mapping")
jsk_pcl_util_nodelet(src/plane_reasoner_nodelet.cpp
  "jsk_pcl_utils/PlaneReasoner" "plane_reasoner")
jsk_pcl_util_nodelet(src/transform_pointcloud_in_bounding_box_nodelet.cpp
  "jsk_pcl_utils/TransformPointcloudInBoundingBox" "transform_pointcloud_in_bounding_box")
jsk_pcl_util_nodelet(src/point_indices_to_cluster_point_indices_nodelet.cpp
  "jsk_pcl_utils/PointIndicesToClusterPointIndices" "point_indices_to_cluster_point_indices")
jsk_pcl_util_nodelet(src/point_indices_to_mask_image_nodelet.cpp
  "jsk_pcl_utils/PointIndicesToMaskImage" "point_indices_to_mask_image")
jsk_pcl_util_nodelet(src/mask_image_to_depth_considered_mask_image_nodelet.cpp
  "jsk_pcl_utils/MaskImageToDepthConsideredMaskImage" "mask_image_to_depth_considered_mask_image")
jsk_pcl_util_nodelet(src/mask_image_to_point_indices_nodelet.cpp
  "jsk_pcl_utils/MaskImageToPointIndices" "mask_image_to_point_indices")
jsk_pcl_util_nodelet(src/label_to_cluster_point_indices_nodelet.cpp
  "jsk_pcl_utils/LabelToClusterPointIndices" "label_to_cluster_point_indices")
jsk_pcl_util_nodelet(src/plane_concatenator_nodelet.cpp
  "jsk_pcl_utils/PlaneConcatenator" "plane_concatenator")
jsk_pcl_util_nodelet(src/add_point_indices_nodelet.cpp
  "jsk_pcl_utils/AddPointIndices" "add_point_indices")
jsk_pcl_util_nodelet(src/subtract_point_indices_nodelet.cpp
  "jsk_pcl_utils/SubtractPointIndices" "subtract_point_indices")
jsk_pcl_util_nodelet(src/pcd_reader_with_pose_nodelet.cpp
  "jsk_pcl_utils/PCDReaderWithPose" "pcd_reader_with_pose")
jsk_pcl_util_nodelet(src/pointcloud_relative_from_pose_stamped_nodelet.cpp
  "jsk_pcl_utils/PointCloudRelativeFromPoseStamped" "pointcloud_relative_from_pose_stamped")
jsk_pcl_util_nodelet(src/pointcloud_to_pcd_nodelet.cpp
  "jsk_pcl_utils/PointCloudToPCD" "pointcloud_to_pcd")
jsk_pcl_util_nodelet(src/marker_array_voxel_to_pointcloud_nodelet.cpp
  "jsk_pcl_utils/MarkerArrayVoxelToPointCloud" "marker_array_voxel_to_pointcloud")
add_library(jsk_pcl_ros_utils SHARED ${jsk_pcl_util_nodelet_sources})
add_dependencies(jsk_pcl_ros_utils ${PROJECT_NAME}_gencfg)
target_link_libraries(jsk_pcl_ros_utils
  ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

catkin_package(
    DEPENDS PCL
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS} sensor_msgs geometry_msgs jsk_recognition_msgs jsk_recognition_utils
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros_utils
)

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
install(TARGETS jsk_pcl_ros_utils
  ${jsk_pcl_util_nodelet_executables}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY sample scripts test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  # Naming Rule: filename of rostest should be {EXECUTABLE_NAME}.test
  add_rostest(test/point_indices_to_cluster_point_indices.test)
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # not build on hydro
    add_rostest(test/add_point_indices.test)
    roslaunch_add_file_check(test/add_point_indices.test)
    add_rostest(test/cloud_on_plane.test)
    add_rostest(test/cloud_on_plane_info.test)
    add_rostest(test/cluster_point_indices_to_point_indices.test)
    add_rostest(test/colorize_distance_from_plane.test)
    add_rostest(test/colorize_height_2d_mapping.test)
    add_rostest(test/delay_pointcloud.test)
    add_rostest(test/depth_image_error.test)
    add_rostest(test/point_indices_to_mask_image.test)
    add_rostest(test/pointcloud_to_pcd.test)
    add_rostest(test/pointcloud_to_point_indices.test)
    add_rostest(test/bounding_box_array_to_bounding_box.test)
    add_rostest(test/evaluate_box_segmentation_by_gt_box.test)
    add_rostest(test/evaluate_voxel_segmentation_by_gt_box.test)
    add_rostest(test/label_to_cluster_point_indices.test)
    add_rostest(test/marker_array_voxel_to_pointcloud.test)
    add_rostest(test/mask_image_to_depth_considered_mask_image.test)
    add_rostest(test/mask_image_to_point_indices.test)
    add_rostest(test/normal_concatenater.test)
    add_rostest(test/normal_flip_to_frame.test)
    add_rostest(test/pcd_reader_with_pose.test)
    add_rostest(test/planar_pointcloud_simulator.test)
    add_rostest(test/plane_concatenator.test)
    add_rostest(test/plane_reasoner.test)
    add_rostest(test/plane_rejector.test)
    add_rostest(test/pointcloud_relative_from_pose_stamped.test)
    add_rostest(test/pointcloud_to_cluster_point_indices.test)
    add_rostest(test/pointcloud_to_mask_image.test)
    add_rostest(test/pointcloud_to_stl.test)
    add_rostest(test/pointcloud_xyz_to_xyzrgb.test)
    add_rostest(test/pointcloud_xyzrgb_to_xyz.test)
    add_rostest(test/polygon_appender.test)
    add_rostest(test/polygon_array_angle_likelihood.test)
    add_rostest(test/polygon_array_area_likelihood.test)
    add_rostest(test/polygon_array_distance_likelihood.test)
    add_rostest(test/polygon_array_foot_angle_likelihood.test)
    add_rostest(test/polygon_array_transformer.test)
    add_rostest(test/polygon_array_unwrapper.test)
    add_rostest(test/polygon_array_likelihood_filter.test)
    add_rostest(test/polygon_array_wrapper.test)
    add_rostest(test/polygon_flipper.test)
    add_rostest(test/polygon_magnifier.test)
    add_rostest(test/polygon_points_sampler.test)
    add_rostest(test/pose_with_covariance_stamped_to_gaussian_pointcloud.test)
    add_rostest(test/spherical_pointcloud_simulator.test)
    add_rostest(test/static_polygon_array_publisher.test)
    add_rostest(test/subtract_point_indices.test)
    add_rostest(test/tf_transform_bounding_box.test)
    add_rostest(test/tf_transform_bounding_box_array.test)
    add_rostest(test/tf_transform_cloud.test)
    add_rostest(test/transform_pointcloud_in_bounding_box.test)
    add_rostest(test/centroid_publisher.test)
    roslaunch_add_file_check(test/subtract_point_indices.test)
  endif()
endif()
