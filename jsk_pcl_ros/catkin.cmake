# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_pcl_ros)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure pcl_ros pcl nodelet message_generation genmsg)
find_package(pcl_ros)

add_message_files(FILES IndicesArray.msg PointsArray.msg)
add_service_files(FILES SwitchTopic.srv  TransformScreenpoint.srv)
generate_messages(DEPENDENCIES pcl)

include_directories(include ${catkin_INCLUDE_DIRS})
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

#find_package(wxWidgets REQUIRED)
#include(${wxWidgets_USE_FILE})
#include_directories( ${wxWidgets_INCLUDE_DIRS} )

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/HSVColorFilter.cfg
  cfg/RGBColorFilter.cfg
  )
include_directories(include cfg/cpp)

catkin_package(
    DEPENDS pcl
    CATKIN-DEPENDS pcl_ros message_runtime
    INCLUDE_DIRS include
    LIBRARIES jsk_pcl_ros
)


# pcl_ros::Filter based class is not working...
# https://github.com/ros-perception/perception_pcl/issues/9
set(SOURCE_FILES
#  src/color_filter_nodelet.cpp
  src/delay_pointcloud_nodelet.cpp
  src/depth_image_creator_nodelet.cpp
#  src/resize_points_publisher.cpp
  src/pointcloud_screenpoint_nodelet.cpp
  )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(jsk_pcl_ros SHARED ${SOURCE_FILES})
target_link_libraries(jsk_pcl_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(jsk_pcl_ros jsk_pcl_ros_gencpp)


add_executable(pointcloud_screenpoint src/pointcloud_screenpoint.cpp)
target_link_libraries(pointcloud_screenpoint ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})

#
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS jsk_pcl_ros pointcloud_screenpoint
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
