if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
file(GLOB _msg_files ${PROJECT_SOURCE_DIR}/msg/*.msg)
foreach(_msg_file ${_msg_files})
  execute_process(COMMAND sed -i s@^pcl_msgs/@pcl/@g ${_msg_file})
endforeach()
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

rosbuild_add_library (jsk_pcl_ros
#  ${pcl_ros_PACKAGE_PATH}/src/pcl_ros/features/feature.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-perception/perception_pcl/issues/9
  src/color_filter_nodelet.cpp
  src/resize_points_publisher.cpp
#  src/color_converter_nodelet.cpp
  src/delay_pointcloud_nodelet.cpp
  src/depth_image_creator_nodelet.cpp
  src/pointcloud_screenpoint_nodelet.cpp
  src/euclidean_cluster_extraction_nodelet.cpp
  src/cluster_point_indices_decomposer_nodelet.cpp
  src/cluster_point_indices_decomposer_z_axis_nodelet.cpp
  src/centroid_publisher_nodelet.cpp
  src/particle_filter_tracking_nodelet.cpp
  #src/pointcloud_flowrate_nodelet.cpp
  src/lazy_concatenater_nodelet.cpp
)

rosbuild_add_compile_flags (jsk_pcl_ros ${SSE_FLAGS})
rosbuild_link_boost (jsk_pcl_ros system)
#target_link_libraries (jsk_pcl_ros tbb pcl_ros_io pcl_ros_features pcl_ros_filters pcl_ros_segmentation pcl_ros_surface)
target_link_libraries (jsk_pcl_ros tbb pcl_ros_io pcl_ros_filters pcl_ros_tf)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(pointcloud_screenpoint src/pointcloud_screenpoint.cpp)
rosbuild_add_executable(centroid_publisher src/centroid_publisher.cpp)
rosbuild_add_executable(particle_filter_tracking src/particle_filter_tracking.cpp)
rosbuild_add_executable(cluster_point_indices_decomposer src/cluster_point_indices_decomposer.cpp)
rosbuild_add_executable(cluster_point_indices_decomposer_z_axis src/cluster_point_indices_decomposer_z_axis.cpp)
rosbuild_add_executable(depth_image_creator src/depth_image_creator.cpp)
rosbuild_add_executable(euclidean_cluster_extraction src/euclidean_cluster_extraction.cpp)
rosbuild_add_executable(icp_server src/icp_server.cpp)
rosbuild_add_executable(pointcloud_flowrate src/pointcloud_flowrate.cpp)
rosbuild_add_executable(lazy_concatenater src/lazy_concatenater.cpp)
# roslaunch documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build --nomakefile RESULT_VARIABLE _make_failed)
