# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(resized_image_transport)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS cv_bridge sensor_msgs image_transport
  std_srvs message_generation dynamic_reconfigure
  nodelet
  jsk_topic_tools)
find_package(OpenCV REQUIRED)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/ImageResizer.cfg
  cfg/LogPolar.cfg
  )

catkin_package(
    DEPENDS # opencv2
    CATKIN_DEPENDS cv_bridge sensor_msgs image_transport std_srvs message_runtime
    INCLUDE_DIRS include # TODO include
    LIBRARIES # TODO
)

if("${jsk_topic_tools_VERSION}" VERSION_LESS 2.0.10) # https://github.com/jsk-ros-pkg/jsk_common/pull/1346 changed since >= 2.0.10
  add_definitions("-DUSE_2_0_9_ADVERTISE_CAMERA")
endif()

jsk_nodelet(src/image_resizer_nodelet.cpp
  "resized_image_transport/ImageResizer"
  "image_resizer"
  nodelet_sources nodelet_executables)
jsk_nodelet(src/log_polar_nodelet.cpp
  "resized_image_transport/LogPolar"
  "log_polar"
  nodelet_sources nodelet_executables)
add_library(resized_image_transport SHARED ${nodelet_sources}
  src/image_processing_nodelet.cpp)
  
add_definitions("-O2 -g")
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(resized_image_transport ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(resized_image_transport ${PROJECT_NAME}_gencfg)


# Mark executables and/or libraries for installation
install(TARGETS image_resizer resized_image_transport ${nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
install(FILES nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# check ros distro >= indigo
execute_process(
  COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('indigo'[0]))"
  RESULT_VARIABLE DISTRO_GE_INDIGO
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if(${DISTRO_GE_INDIGO})
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    add_rostest(test/image_resizer.test)
  endif()
endif()
