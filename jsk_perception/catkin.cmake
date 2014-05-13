cmake_minimum_required(VERSION 2.8.3)
project(jsk_perception)

find_package(catkin REQUIRED COMPONENTS message_generation imagesift std_msgs sensor_msgs geometry_msgs cv_bridge image_geometry image_transport driver_base dynamic_reconfigure pcl_ros eigen roscpp nodelet rostest)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system signals)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/camshiftdemo.cfg cfg/EdgeDetector.cfg cfg/HoughLines.cfg cfg/matchtemplate.cfg cfg/point_pose_extractor.cfg cfg/RectangleDetector.cfg)

add_message_files(FILES # ClusterPointIndices.msg
      PointsArray.msg RotatedRectStamped.msg LineArray.msg Rect.msg Line.msg RotatedRect.msg SparseImage.msg)

add_service_files(FILES EuclideanSegment.srv  SetTemplate.srv  WhiteBalancePoints.srv  WhiteBalance.srv)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(camshiftdemo src/camshiftdemo.cpp)
add_executable(virtual_camera_mono src/virtual_camera_mono.cpp)
add_executable(point_pose_extractor src/point_pose_extractor.cpp)
add_executable(white_balance_converter src/white_balance_converter.cpp)
add_executable(hough_lines src/hough_lines.cpp)
add_executable(rectangle_detector src/rectangle_detector.cpp)

macro(jsk_perception_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  list(APPEND jsk_perception_nodelet_sources ${_nodelet_cpp})
  set(NODELET ${_nodelet_class})
  set(DEFAULT_NODE_NAME ${_single_nodelet_exec_name})
  configure_file(${PROJECT_SOURCE_DIR}/src/single_nodelet_exec.cpp.in
    ${_single_nodelet_exec_name}.cpp)
  add_executable(${_single_nodelet_exec_name} ${_single_nodelet_exec_name}.cpp)
  target_link_libraries(${_single_nodelet_exec_name}
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(${_single_nodelet_exec_name}
    ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
  install(TARGETS ${_single_nodelet_exec_name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endmacro()
jsk_perception_nodelet(src/edge_detector.cpp "jsk_perception/EdgeDetector" "edge_detector")
jsk_perception_nodelet(src/sparse_image_encoder.cpp "jsk_perception/SparseImageEncoder" "sparse_image_encoder")
jsk_perception_nodelet(src/sparse_image_decoder.cpp "jsk_perception/SparseImageDecoder" "sparse_image_decoder")

# compiling jsk_perception library for nodelet
add_library(${PROJECT_NAME} SHARED ${jsk_perception_nodelet_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)


target_link_libraries(camshiftdemo             ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(virtual_camera_mono      ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(point_pose_extractor     ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} /opt/ros/$ENV{ROS_DISTRO}/lib/librospack.so)
target_link_libraries(edge_detector            ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(white_balance_converter  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(hough_lines              ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(rectangle_detector       ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(camshiftdemo             ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(virtual_camera_mono      ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(point_pose_extractor     ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp libsiftfast)
add_dependencies(white_balance_converter  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(hough_lines              ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(rectangle_detector       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

#add_custom_command(
#  OUTPUT  ${PROJECT_SOURCE_DIR}/template
#  DEPENDS ${PROJECT_SOURCE_DIR}/src/eusmodel_template_gen.l
#  COMMAND ${PROJECT_SOURCE_DIR}/src/eusmodel_template_gen.sh)
#add_custom_target(eusmodel_template ALL DEPENDS ${PROJECT_SOURCE_DIR}/template)

install(TARGETS camshiftdemo virtual_camera_mono point_pose_extractor white_balance_converter hough_lines rectangle_detector ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY sample launch euslisp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(FILES jsk_perception_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_rostest(test/sparse_image.test)
