# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(checkerboard_detector)

find_package(catkin REQUIRED COMPONENTS roscpp rosconsole dynamic_reconfigure cv_bridge sensor_msgs image_geometry
  posedetection_msgs eigen_conversions message_filters tf tf2 jsk_recognition_msgs jsk_recognition_msgs
)
find_package(OpenMP)
find_package(posedetection_msgs)

generate_dynamic_reconfigure_options(cfg/CheckerboardDetector.cfg)

catkin_package(
    CATKIN_DEPENDS roscpp rosconsole cv_bridge sensor_msgs posedetection_msgs image_geometry jsk_recognition_msgs
    DEPENDS
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(checkerboard_detector src/checkerboard_detector.cpp)
target_link_libraries(checkerboard_detector ${catkin_LIBRARIES})
add_executable(objectdetection_transform_echo src/objectdetection_transform_echo.cpp)
target_link_libraries(objectdetection_transform_echo ${catkin_LIBRARIES})
add_executable(checkerboard_calibration src/checkerboard_calibration.cpp)
target_link_libraries(checkerboard_calibration ${catkin_LIBRARIES})
add_dependencies(checkerboard_detector    posedetection_msgs_generate_messages_cpp)
add_dependencies(checkerboard_calibration posedetection_msgs_generate_messages_cpp)
add_dependencies(objectdetection_transform_echo posedetection_msgs_generate_messages_cpp)
add_dependencies(checkerboard_detector    jsk_recognition_msgs_generate_messages_cpp)
add_dependencies(checkerboard_calibration jsk_recognition_msgs_generate_messages_cpp)
add_dependencies(objectdetection_transform_echo jsk_recognition_msgs_generate_messages_cpp)
add_dependencies(checkerboard_detector ${PROJECT_NAME}_gencfg)

install(TARGETS checkerboard_detector checkerboard_calibration
  objectdetection_transform_echo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/objectdetection_tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB _launch *.launch)
file(GLOB _pdf *.pdf)
install(FILES ${_launch} ${_pdf}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY sample test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    add_rostest(test/checkerboard_detector.test)
    add_rostest(test/objectdetection_tf_publisher.test)
    add_rostest(test/objectdetection_transform_echo.test)
  endif()
endif()
