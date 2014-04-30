# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  # update package.xml, in groovy we need to add pcl to package.xml
  execute_process(COMMAND sed -i s@<run_depend>pcl_ros</run_depend>@<run_depend>pcl_ros</run_depend><run_depend>pcl</run_depend>@g ${PROJECT_SOURCE_DIR}/package.xml)
  execute_process(COMMAND sed -i s@<build_depend>pcl_ros</build_depend>@<build_depend>pcl_ros</build_depend><build_depend>pcl</build_depend>@g ${PROJECT_SOURCE_DIR}/package.xml)
endif($ENV{ROS_DISTRO} STREQUAL "groovy")


# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(PCL_MSGS pcl)
else()
  set(PCL_MSGS pcl_msgs) ## hydro and later
endif()
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure pcl_ros nodelet message_generation genmsg ${PCL_MSGS} sensor_msgs geometry_msgs
  eigen_conversions tf_conversions tf2_ros tf image_transport nodelet cv_bridge)

add_message_files(FILES IndicesArray.msg PointsArray.msg ClusterPointIndices.msg Int32Stamped.msg SnapItRequest.msg PolygonArray.msg
  SlicedPointCloud.msg)
add_service_files(FILES SwitchTopic.srv  TransformScreenpoint.srv CheckCircle.srv RobotPickupReleasePoint.srv  TowerPickUp.srv EuclideanSegment.srv TowerRobotMoveCommand.srv SetPointCloud2.srv
  CallSnapIt.srv CallPolygon.srv)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/HSVColorFilter.cfg
  cfg/RGBColorFilter.cfg
  cfg/ImageRotate.cfg
  )

find_package(OpenCV REQUIRED core imgproc)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

macro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  list(APPEND jsk_pcl_nodelet_sources ${_nodelet_cpp})
  set(NODELET ${_nodelet_class})
  set(DEFAULT_NODE_NAME ${_single_nodelet_exec_name})
  configure_file(${PROJECT_SOURCE_DIR}/src/single_nodelet_exec.cpp.in
    ${_single_nodelet_exec_name}.cpp)
  add_executable(${_single_nodelet_exec_name} ${_single_nodelet_exec_name}.cpp)
  target_link_libraries(${_single_nodelet_exec_name}
    ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
  add_dependencies(${_single_nodelet_exec_name}
    ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
  install(TARGETS ${_single_nodelet_exec_name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endmacro(jsk_pcl_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
jsk_pcl_nodelet(src/pointcloud_screenpoint_nodelet.cpp "jsk_pcl/PointcloudScreenpoint" "pointcloud_screenpoint")
jsk_pcl_nodelet(src/particle_filter_tracking_nodelet.cpp "jsk_pcl/ParticleFilterTracking" "particle_filter_tracking")
jsk_pcl_nodelet(src/voxel_grid_downsample_manager_nodelet.cpp "jsk_pcl/VoxelGridDownsampleManager" "voxel_grid_downsample_manager")
jsk_pcl_nodelet(src/voxel_grid_downsample_decoder_nodelet.cpp "jsk_pcl/VoxelGridDownsampleDecoder" "voxel_grid_downsample_decoder")
jsk_pcl_nodelet(src/snapit_nodelet.cpp "jsk_pcl/Snapit" "snapit")
jsk_pcl_nodelet(src/keypoints_publisher_nodelet.cpp "jsk_pcl/KeypointsPublisher" "keypoints_publisher")
jsk_pcl_nodelet(src/hinted_plane_detector_nodelet.cpp "jsk_pcl/HintedPlaneDetector" "hinted_plane_detector")
jsk_pcl_nodelet(src/pointcloud_throttle_nodelet.cpp "jsk_pcl/NodeletPointCloudThrottle" "point_cloud_throttle")
jsk_pcl_nodelet(src/image_throttle_nodelet.cpp
  "jsk_pcl/NodeletImageThrottle" "image_throttle")
jsk_pcl_nodelet(src/image_mux_nodelet.cpp
  "jsk_pcl/NodeletImageMUX" "image_mux")
jsk_pcl_nodelet(src/image_demux_nodelet.cpp
  "jsk_pcl/NodeletImageDEMUX" "image_demux")
jsk_pcl_nodelet(src/image_rotate_nodelet.cpp
  "jsk_pcl/ImageRotateNodelet" "image_rotate")
jsk_pcl_nodelet(src/octree_change_publisher_nodelet.cpp
  "jsk_pcl/OctreeChangePublisher" "octree_change_publisher")
jsk_pcl_nodelet(src/tf_transform_cloud_nodelet.cpp
  "jsk_pcl/TfTransformCloud" "tf_transform_cloud")


add_library(jsk_pcl_ros SHARED ${jsk_pcl_nodelet_sources})
target_link_libraries(jsk_pcl_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(jsk_pcl_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

generate_messages(DEPENDENCIES ${PCL_MSGS} sensor_msgs geometry_msgs)

catkin_package(
    DEPENDS pcl
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS} sensor_msgs geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS jsk_pcl_ros
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
