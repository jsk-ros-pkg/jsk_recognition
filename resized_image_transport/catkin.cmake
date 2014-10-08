# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(resized_image_transport)

find_package(catkin REQUIRED COMPONENTS cv_bridge sensor_msgs image_transport
  std_srvs message_generation dynamic_reconfigure
  nodelet
  jsk_topic_tools)
find_package(OpenCV REQUIRED)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/ImageResizer.cfg
  )

catkin_package(
    DEPENDS # opencv2
    CATKIN_DEPENDS cv_bridge sensor_msgs image_transport std_srvs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

if(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
else(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_PREFIX}/share/jsk_topic_tools/cmake/nodelet.cmake)
endif(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)

jsk_nodelet(src/image_resizer_nodelet.cpp
  "resized_image_transport/ImageResizer"
  "image_resizer"
  nodelet_sources nodelet_executables)
add_library(resized_image_transport SHARED ${nodelet_sources})
  
add_definitions("-O2 -g")
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(resized_image_transport ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(resized_image_transport ${PROJECT_NAME}_gencfg)


# Mark executables and/or libraries for installation
install(TARGETS image_resizer resized_image_transport ${nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
install(FILES nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
