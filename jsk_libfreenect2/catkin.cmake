cmake_minimum_required(VERSION 2.8.3)
project(jsk_libfreenect2)

find_package(catkin REQUIRED COMPONENTS
  mk
  cv_bridge
  roscpp
  sensor_msgs
  nodelet
  image_transport
  camera_info_manager
  jsk_topic_tools
)

catkin_package(
#  INCLUDE_DIRS include ${libfreenect_source_dir}/include
  INCLUDE_DIRS include
  LIBRARIES freenect2 freenect2_nodelet
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

LINK_DIRECTORIES(${CATKIN_DEVEL_PREFIX}/lib ${CATKIN_DEVEL_PREFIX}/lib64)
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/installed)
  execute_process(
    COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}
    make -f ${PROJECT_SOURCE_DIR}/Makefile.libfreenect2
    MK_DIR=${mk_PREFIX}/share/mk
    INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
    installed
    RESULT_VARIABLE _make_failed)
  if (_make_failed)
    message(FATAL_ERROR "Compile libfreenect2 failed")
  endif(_make_failed)
endif(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/installed)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)

SET(libfreenect_source_dir
  ${CMAKE_CURRENT_BINARY_DIR}/build/libfreenect2/examples/protonect)
SET(RESOURCES_INC_FILE "${libfreenect_source_dir}/src/resources.inc")
LIST(APPEND CMAKE_MODULE_PATH ${libfreenect_source_dir}/cmake_modules)
INCLUDE(SetupLibfreenect2Threading)

INCLUDE_DIRECTORIES(${libfreenect_source_dir}/${LIBFREENECT2_THREADING_INCLUDE_DIR})

# INCLUDE(${libfreenect_source_dir}/cmake_modules/GenerateResources.cmake)
FUNCTION(GENERATE_RESOURCES OUTPUT BASE_FOLDER)
  ADD_EXECUTABLE(generate_resources_tool
    ${libfreenect_source_dir}/src/generate_resources.cpp
    )
  ADD_CUSTOM_COMMAND(
    OUTPUT ${OUTPUT}
    COMMAND generate_resources_tool ${BASE_FOLDER} ${ARGN} > ${OUTPUT}
    WORKING_DIRECTORY ${BASE_FOLDER}
    DEPENDS generate_resources_tool #${ARGN}
    )
ENDFUNCTION(GENERATE_RESOURCES)

GENERATE_RESOURCES(${RESOURCES_INC_FILE} ${libfreenect_source_dir}
  11to16.bin
  xTable.bin
  zTable.bin
  src/shader/debug.fs
  src/shader/default.vs
  src/shader/filter1.fs
  src/shader/filter2.fs
  src/shader/stage1.fs
  src/shader/stage2.fs
)

INCLUDE_DIRECTORIES(${libfreenect_source_dir}/include)
INCLUDE_DIRECTORIES(${CATKIN_DEVEL_PREFIX}/include/libusb-1.0)
ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)
ADD_DEFINITIONS(-DRESOURCES_INC)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS}/GLFW)

ADD_LIBRARY(freenect2 SHARED
  ${libfreenect_source_dir}/src/opengl.cpp
  ${libfreenect_source_dir}/src/transfer_pool.cpp
  ${libfreenect_source_dir}/src/event_loop.cpp

  ${libfreenect_source_dir}/src/double_buffer.cpp
  ${libfreenect_source_dir}/src/frame_listener_impl.cpp

  ${libfreenect_source_dir}/src/rgb_packet_stream_parser.cpp
  ${libfreenect_source_dir}/src/rgb_packet_processor.cpp
  ${libfreenect_source_dir}/src/turbo_jpeg_rgb_packet_processor.cpp

  ${libfreenect_source_dir}/src/depth_packet_stream_parser.cpp
  ${libfreenect_source_dir}/src/depth_packet_processor.cpp
  ${libfreenect_source_dir}/src/cpu_depth_packet_processor.cpp
  ${libfreenect_source_dir}/src/opengl_depth_packet_processor.cpp
  ${libfreenect_source_dir}/src/resource.cpp

  ${libfreenect_source_dir}/src/usb_control.cpp
  ${libfreenect_source_dir}/src/command_transaction.cpp
  ${libfreenect_source_dir}/src/libfreenect2.cpp
  ${libfreenect_source_dir}/${LIBFREENECT2_THREADING_SOURCE}
  ${RESOURCES_INC_FILE}
)

TARGET_LINK_LIBRARIES(freenect2
  glfw GLEW GLEWmx usb-1.0 turbojpeg
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
  )

if(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
else(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)
  include(${jsk_topic_tools_PREFIX}/share/jsk_topic_tools/cmake/nodelet.cmake)
endif(EXISTS ${jsk_topic_tools_SOURCE_DIR}/cmake/nodelet.cmake)

macro(freenect_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    freenect_nodelet_sources freenect_nodelet_executables)
endmacro(freenect_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

freenect_nodelet(src/driver_nodelet.cpp "jsk_libfreenect2/Driver" "driver")

add_definitions("-O2 -g")
include_directories(include)
add_library(freenect2_nodelet SHARED ${freenect_nodelet_sources})
target_link_libraries(freenect2_nodelet
  freenect2
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(freenect2_nodelet ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


# install
install(DIRECTORY
  include/${PROJECT_NAME}/
  ${CATKIN_DEVEL_PREFIX}/include/libusb-1.0
  ${CATKIN_DEVEL_PREFIX}/include/GL
  ${CATKIN_DEVEL_PREFIX}/include/GLFW
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS freenect2 freenect2_nodelet driver ${freenect_nodelet_executables}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
  ${CATKIN_DEVEL_PREFIX}/lib/libusb-1.0.a
  ${CATKIN_DEVEL_PREFIX}/lib/libusb-1.0.la
  ${CATKIN_DEVEL_PREFIX}/lib/libusb-1.0.so
  ${CATKIN_DEVEL_PREFIX}/lib/libusb-1.0.so.0
  ${CATKIN_DEVEL_PREFIX}/lib/libusb-1.0.so.0.1.0
  ${CATKIN_DEVEL_PREFIX}/lib/libglfw.so
  ${CATKIN_DEVEL_PREFIX}/lib/libglfw.so.3
  ${CATKIN_DEVEL_PREFIX}/lib/libglfw.so.3.0
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEW.a
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEWmx.a
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEWmx.so
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEWmx.so.1.11
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEWmx.so.1.11.0
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEW.so
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEW.so.1.11
  ${CATKIN_DEVEL_PREFIX}/lib64/libGLEW.so.1.11.0
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# pkgconfig is left not installed

