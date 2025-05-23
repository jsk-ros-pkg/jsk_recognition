cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_utils)

# Use ccache if installed to make it fast to generate object files
if (${CMAKE_VERSION} VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif()
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  jsk_recognition_msgs
  pcl_ros
  pcl_msgs
  eigen_conversions
  message_generation
  rostest
  tf_conversions
  tf
  tf2_ros
  image_geometry
  sensor_msgs
  geometry_msgs
  jsk_topic_tools
  visualization_msgs
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/jsk_recognition_utils/jsk_topic_tools_version.h.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/jsk_recognition_utils/jsk_topic_tools_version.h)


# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install font data
add_custom_target(${PROJECT_NAME}_install_font_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_font_data.py)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/PoseArrayToPose.cfg
  cfg/PolygonArrayToBoxArray.cfg
  cfg/PolygonArrayToPolygon.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jsk_recognition_utils
 CATKIN_DEPENDS cv_bridge jsk_recognition_msgs pcl_ros visualization_msgs message_runtime
)

# Cythonize pyx files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(UseCython)
add_subdirectory(python/${PROJECT_NAME})

find_package(OpenCV REQUIRED core imgproc)
find_package(PCL REQUIRED)
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
IF(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
ENDIF()
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
link_libraries(${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)

add_library(jsk_recognition_utils SHARED
  src/grid_index.cpp
  src/grid_map.cpp
  src/grid_line.cpp
  src/geo_util.cpp
  src/random_util.cpp
  src/pcl_ros_util.cpp
  src/pcl_conversion_util.cpp
  src/pcl_util.cpp
  src/tf_listener_singleton.cpp
  src/pcl/ear_clipping_patched.cpp
  src/sensor_model_utils.cpp
  src/cv_utils.cpp
  src/rgb_colors.cpp
  src/geo/line.cpp
  src/geo/segment.cpp
  src/geo/polyline.cpp
  src/geo/plane.cpp
  src/geo/polygon.cpp
  src/geo/convex_polygon.cpp
  src/geo/cube.cpp
  src/geo/cylinder.cpp
  src/geo/grid_plane.cpp
  src/time_util.cpp
  )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo") # kinetic and later uses qt5
    find_package(Qt5Widgets REQUIRED)
    set(QT_LIBRARIES Qt5::Widgets)
  endif()
  add_rostest_gtest(test_tf_listener_singleton test/tf_listener_singleton.test src/tests/test_tf_listener_singleton.cpp)
  target_link_libraries(test_tf_listener_singleton ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(test_cv_utils src/tests/test_cv_utils.cpp)
  target_link_libraries(test_cv_utils ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  catkin_add_gtest(test_rgb_colors src/tests/test_rgb_colors.cpp)
  target_link_libraries(test_rgb_colors ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    add_rostest(sample/sample_add_bounding_box_array.launch ARGS gui:=false)
    add_rostest(sample/sample_bounding_box_array_publisher.launch ARGS gui:=false)
    add_rostest(test/add_cluster_indices.test)
    add_rostest(test/image_16uc1_to_32fc1.test)
    add_rostest(test/polygon_array_publisher.test)
    add_rostest(test/polygon_array_to_box_array.test)
    add_rostest(test/polygon_array_to_polygon.test)
    add_rostest(test/pose_array_to_pose.test)
    add_rostest(test/rect_array_to_polygon_array.test)
    add_rostest(test/static_virtual_camera.test)
  endif()
endif()

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(TARGETS jsk_recognition_utils
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" # Ignore *_version.h.in files
)

install(DIRECTORY sample test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
install(DIRECTORY node_scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
else()
install(DIRECTORY node_scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*"
  PATTERN "*/*.py" EXCLUDE
  )
file(GLOB NODE_SCRIPT_PROGRAMS node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)
endif()
