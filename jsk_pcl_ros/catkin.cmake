# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  set(PCL_MSGS pcl)
else()
  set(PCL_MSGS pcl_msgs) ## hydro and later
endif()
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure pcl_ros nodelet message_generation genmsg ${PCL_MSGS})

add_message_files(FILES IndicesArray.msg PointsArray.msg ClusterPointIndices.msg Int32Stamped.msg SnapItRequest.msg PolygonArray.msg)
add_service_files(FILES SwitchTopic.srv  TransformScreenpoint.srv CheckCircle.srv RobotPickupReleasePoint.srv  TowerPickUp.srv EuclideanSegment.srv TowerRobotMoveCommand.srv SetPointCloud2.srv
  CallSnapIt.srv)
generate_messages(DEPENDENCIES ${PCL_MSGS})

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/HSVColorFilter.cfg
  cfg/RGBColorFilter.cfg
  )

if($ENV{ROS_DISTRO} STREQUAL "groovy")
catkin_package(
    DEPENDS pcl
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS}
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros
)
else($ENV{ROS_DISTRO} STREQUAL "groovy")
catkin_package(
    CATKIN_DEPENDS pcl_ros message_runtime ${PCL_MSGS} pcl
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros
)
  
endif($ENV{ROS_DISTRO} STREQUAL "groovy")
#include_directories(include cfg/cpp)
include_directories(include ${catkin_INCLUDE_DIRS})


# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
set(SOURCE_FILES
#  src/color_filter_nodelet.cpp
  src/delay_pointcloud_nodelet.cpp
  src/depth_image_creator_nodelet.cpp
#  src/resize_points_publisher.cpp
  src/pointcloud_screenpoint_nodelet.cpp
  src/particle_filter_tracking_nodelet.cpp
  src/voxel_grid_downsample_manager_nodelet.cpp
  src/voxel_grid_downsample_decoder_nodelet.cpp
  src/snapit_nodelet.cpp
  src/keypoints_publisher_nodelet.cpp
  src/hinted_plane_detector_nodelet.cpp
  )


add_library(jsk_pcl_ros SHARED ${SOURCE_FILES})
target_link_libraries(jsk_pcl_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(jsk_pcl_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


add_executable(pointcloud_screenpoint src/pointcloud_screenpoint.cpp)
target_link_libraries(pointcloud_screenpoint ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(pointcloud_screenpoint ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(particle_filter_tracking src/particle_filter_tracking.cpp)
target_link_libraries(particle_filter_tracking ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(particle_filter_tracking ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


#
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS jsk_pcl_ros pointcloud_screenpoint particle_filter_tracking
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
