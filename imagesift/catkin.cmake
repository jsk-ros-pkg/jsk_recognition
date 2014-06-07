cmake_minimum_required(VERSION 2.8.3)
project(imagesift)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs posedetection_msgs image_transport cv_bridge)

find_package(OpenCV)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs posedetection_msgs image_transport cv_bridge 
    LIBRARIES
    INCLUDE_DIRS
    DEPENDS OpenCV libsiftfast
)


set(ENV{PKG_CONFIG_PATH} ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
find_package(PkgConfig REQUIRED)
pkg_check_modules(siftfast libsiftfast REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${siftfast_INCLUDE_DIRS})
link_directories(${siftfast_LIBRARY_DIRS})
link_libraries(${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${siftfast_LIBRARIES})


macro(jsk_feature detector extractor exec_name)
  list(APPEND jsk_exec ${exec_name})
  set(DETECTOR ${detector})
  set(EXTRACTOR ${extractor})
  configure_file(imagefeatures.cpp.in ${exec_name}.cpp) #${CMAKE_CURRENT_BINARY_DIR}/
  add_executable(${exec_name} ${exec_name}.cpp)
  set_target_properties(${exec_name} PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
  add_dependencies(${exec_name} posedetection_msgs_generate_messages_cpp libsiftfast)
endmacro(jsk_feature detector extractor exec_name)

jsk_feature("SURF" "SURF" "imagesurf")
jsk_feature("STAR" "SURF" "imagestar")
jsk_feature("BRISK" "BRISK" "imagebrisk")

add_executable(imagesift imagesift.cpp)
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
add_dependencies(imagesift libsiftfast)
add_dependencies(imagesift posedetection_msgs_generate_messages_cpp libsiftfast)


install(TARGETS imagesift ${jsk_exec} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
