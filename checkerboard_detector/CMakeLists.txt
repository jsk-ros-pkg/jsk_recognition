if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
#set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src)
