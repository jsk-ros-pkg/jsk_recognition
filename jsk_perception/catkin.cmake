cmake_minimum_required(VERSION 2.8.3)
project(jsk_perception)

find_package(catkin REQUIRED COMPONENTS message_generation imagesift std_msgs sensor_msgs geometry_msgs cv_bridge image_geometry image_transport driver_base dynamic_reconfigure pcl_ros eigen roscpp)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system signals)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/camshiftdemo.cfg cfg/EdgeDetector.cfg cfg/HoughLines.cfg cfg/matchtemplate.cfg cfg/point_pose_extractor.cfg cfg/RectangleDetector.cfg)

add_message_files(FILES # ClusterPointIndices.msg
      PointsArray.msg RotatedRectStamped.msg LineArray.msg Rect.msg Line.msg RotatedRect.msg)

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
add_executable(edge_detector src/edge_detector.cpp)
add_executable(hough_lines src/hough_lines.cpp)
add_executable(rectangle_detector src/rectangle_detector.cpp)

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
add_dependencies(edge_detector            ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(white_balance_converter  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(hough_lines              ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(rectangle_detector       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

#add_custom_command(
#  OUTPUT  ${PROJECT_SOURCE_DIR}/template
#  DEPENDS ${PROJECT_SOURCE_DIR}/src/eusmodel_template_gen.l
#  COMMAND ${PROJECT_SOURCE_DIR}/src/eusmodel_template_gen.sh)
#add_custom_target(eusmodel_template ALL DEPENDS ${PROJECT_SOURCE_DIR}/template)

install(TARGETS camshiftdemo virtual_camera_mono point_pose_extractor white_balance_converter edge_detector hough_lines rectangle_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY sample launch euslisp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)


