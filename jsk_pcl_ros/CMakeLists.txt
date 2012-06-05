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

rosbuild_add_library (jsk_pcl_ros
  src/color_filter_nodelet.cpp
  src/color_converter_nodelet.cpp
  src/delay_pointcloud_nodelet.cpp
  src/HSV_color_filter_nodelet.cpp
  src/resize_points_publisher.cpp
  src/pointcloud_screenpoint_nodelet.cpp)

rosbuild_add_compile_flags (jsk_pcl_ros ${SSE_FLAGS})
rosbuild_link_boost (jsk_pcl_ros system)
target_link_libraries (jsk_pcl_ros tbb pcl_ros_io pcl_ros_features pcl_ros_filters pcl_ros_segmentation pcl_ros_surface)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(pointcloud_screenpoint src/pointcloud_screenpoint.cpp)

# roslaunch documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)
