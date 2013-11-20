# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(resized_image_transport)

find_package(catkin REQUIRED COMPONENTS cv_bridge sensor_msgs image_transport std_srvs message_generation dynamic_reconfigure)
find_package(OpenCV REQUIRED)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/ImageResizer.cfg
  )

catkin_package(
    DEPENDS # opencv2
    CATKIN-DEPENDS cv_bridge sensor_msgs image_transport std_srvs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(image_resizer src/image_resizer.cpp)
target_link_libraries(image_resizer ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(image_resizer ${PROJECT_NAME}_gencfg)


# Mark executables and/or libraries for installation
install(TARGETS image_resizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


