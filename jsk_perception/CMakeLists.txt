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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
rosbuild_add_executable(camshiftdemo src/camshiftdemo.cpp)
rosbuild_add_executable(virtual_camera_mono src/virtual_camera_mono.cpp)
rosbuild_add_executable(point_pose_extractor src/point_pose_extractor.cpp)
rosbuild_add_executable(white_balance_converter src/white_balance_converter.cpp)

rosbuild_add_executable(euclidean_clustering src/euclidean_clustering.cpp)
rosbuild_add_executable(plane_detector src/plane_detector.cpp)
rosbuild_add_executable(color_extractor src/color_extractor.cpp)

target_link_libraries(camshiftdemo ${OpenCV_LIBRARIES})
target_link_libraries(virtual_camera_mono ${OpenCV_LIBRARIES} rospack)
target_link_libraries(point_pose_extractor ${OpenCV_LIBRARIES} rospack)
rosbuild_link_boost(point_pose_extractor filesystem system)

ADD_CUSTOM_COMMAND(
  OUTPUT  ${CMAKE_SOURCE_DIR}/template
  DEPENDS ${CMAKE_SOURCE_DIR}/src/eusmodel_template_gen.l
  COMMAND ${CMAKE_SOURCE_DIR}/src/eusmodel_template_gen.sh)
ADD_CUSTOM_TARGET(eusmodel_template ALL DEPENDS ${CMAKE_SOURCE_DIR}/template)

