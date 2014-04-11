if(NOT USE_ROSBUILD)

cmake_minimum_required(VERSION 2.8.3)
project(imagesift)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs posedetection_msgs image_transport cv_bridge libsiftfast)

find_package(OpenCV)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs posedetection_msgs image_transport cv_bridge libsiftfast
    LIBRARIES
    INCLUDE_DIRS
    DEPENDS OpenCV
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(imagesift imagesift.cpp)
add_executable(imagestar imagestar.cpp)
add_executable(imagesurf imagesurf.cpp)
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
set_target_properties(imagestar PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
target_link_libraries(imagesift ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(imagesurf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(imagestar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(imagesift posedetection_msgs_generate_messages_cpp libsiftfast)
add_dependencies(imagestar posedetection_msgs_generate_messages_cpp libsiftfast)
add_dependencies(imagesurf posedetection_msgs_generate_messages_cpp libsiftfast)

install(TARGETS imagesift imagestar imagesurf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else()

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_add_executable(imagesift imagesift.cpp)
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
rosbuild_add_executable(imagesurf imagesurf.cpp)
set_target_properties(imagesift PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
rosbuild_add_executable(imagestar imagestar.cpp)
set_target_properties(imagestar PROPERTIES COMPILE_FLAGS "-msse2 -O3" LINK_FLAGS "-msse2 -O3")
find_package(OpenCV REQUIRED)
target_link_libraries(imagesift ${OpenCV_LIBRARIES})
target_link_libraries(imagesurf ${OpenCV_LIBRARIES})
target_link_libraries(imagestar ${OpenCV_LIBRARIES})

endif()
