if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
                                                                               
macro(jsk_feature detector extractor exec_name)
  set(DETECTOR ${detector})                                               
  set(EXTRACTOR ${extractor})                                                 
  configure_file(${PROJECT_SOURCE_DIR}/imagefeatures.cpp.in ${exec_name}.cpp) #${CMAKE_CURRENT_BINARY_DIR}/
  rosbuild_add_executable(${exec_name} build/${exec_name}.cpp)
  set_target_properties(${exec_name} PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
  target_link_libraries(${exec_name} ${OpenCV_LIBRARIES})
  add_dependencies(${exec_name} posedetection_msgs_generate_messages_cpp libsiftfast)
endmacro(jsk_feature detector extractor exec_name)

jsk_feature("SURF" "SURF" "imagesurf")
jsk_feature("STAR" "SURF" "imagestar")
jsk_feature("BRISK" "BRISK" "imagebrisk")
jsk_feature("SIFT" "SIFT" "imagesift_sift")
jsk_feature("SIFT" "SURF" "imagesift_surf")

rosbuild_add_executable(imagesift imagesift.cpp)
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
target_link_libraries(imagesift ${OpenCV_LIBRARIES})
