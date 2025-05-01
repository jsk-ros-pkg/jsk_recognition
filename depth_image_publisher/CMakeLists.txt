cmake_minimum_required(VERSION 2.8.3)
project(depth_image_publisher)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure camera_info_manager sensor_msgs image_transport nodelet roscpp)

find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
if(OpenCV_VERSION VERSION_LESS "3.0.0")
  find_package(OpenCV 2 REQUIRED)
elseif(OpenCV_VERSION VERSION_LESS "4.0.0")
  find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs videoio)
else()
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs videoio)
endif()

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/DepthImagePublisher.cfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/nodelet/depth_image_publisher_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS depth_image_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(depth_image_publisher_exe src/node/depth_image_publisher.cpp)
SET_TARGET_PROPERTIES(depth_image_publisher_exe PROPERTIES OUTPUT_NAME depth_image_publisher)
target_link_libraries(depth_image_publisher_exe ${catkin_LIBRARIES})

install(TARGETS depth_image_publisher_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest(test/depth_image_publisher.test)
  roslaunch_add_file_check(test/depth_image_publisher.test)
endif()
