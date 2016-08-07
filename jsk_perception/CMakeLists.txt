cmake_minimum_required(VERSION 2.8.3)
project(jsk_perception)

# ------------------------------------------------------------------------------------
# Initialization
#  1. Resolve dependencies with find_package
#  2. Set build flags
# ------------------------------------------------------------------------------------

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  cv_bridge
  driver_base
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  image_view2
  jsk_recognition_msgs
  jsk_recognition_utils
  jsk_topic_tools
  message_generation
  mk
  nodelet
  pcl_ros
  posedetection_msgs
  roscpp
  roseus
  rospack
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS filesystem system signals)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

find_package(Eigen REQUIRED)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
IF(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
ENDIF()

find_package(OpenMP)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
add_definitions("-g -O2")
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install sample data
add_custom_target(install_sample_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
# download and install trained data
add_custom_target(install_trained_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

execute_process(
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}
  make -f ${PROJECT_SOURCE_DIR}/Makefile.slic
  INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
  MK_DIR=${mk_PREFIX}/share/mk installed
  RESULT_VARIABLE _make_failed)

# ------------------------------------------------------------------------------------
# Catkin setup
# ------------------------------------------------------------------------------------

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/RectArrayActualSizeFilter.cfg
  cfg/PolygonArrayColorLikelihood.cfg
  cfg/PolygonArrayColorHistogram.cfg
  cfg/TabletopColorDifferenceLikelihood.cfg
  cfg/ProjectImagePoint.cfg
  cfg/MaskImageGenerator.cfg
  cfg/MorphologicalMaskImageOperator.cfg
  cfg/BlobDetector.cfg
  cfg/SingleChannelHistogram.cfg
  cfg/ColorHistogramLabelMatch.cfg
  cfg/GridLabel.cfg
  cfg/SLICSuperPixels.cfg
  cfg/SnakeSegmentation.cfg
  cfg/camshiftdemo.cfg
  cfg/EdgeDetector.cfg
  cfg/HoughLines.cfg
  cfg/matchtemplate.cfg
  cfg/point_pose_extractor.cfg
  cfg/RectangleDetector.cfg
  cfg/ColorHistogram.cfg
  cfg/HoughCircles.cfg
  cfg/ColorHistogramSlidingMatcher.cfg
  cfg/BackgroundSubstraction.cfg
  cfg/GrabCut.cfg
  cfg/Fisheye.cfg
  cfg/SlidingWindowObjectDetector.cfg
  cfg/ImagePublisher.cfg
  cfg/ExtractImageChannel.cfg
  cfg/GaussianBlur.cfg
  cfg/KMeans.cfg
  cfg/ImageTimeDiff.cfg
  cfg/OverlayImageColorOnMono.cfg
  cfg/SelectiveSearch.cfg
  cfg/LabelToMaskImage.cfg
)

add_service_files(FILES EuclideanSegment.srv  SetTemplate.srv  WhiteBalancePoints.srv  WhiteBalance.srv NonMaximumSuppression.srv)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime jsk_recognition_msgs jsk_recognition_utils
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES jsk_perception oriented_gradient
)

# ------------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------------

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/build/patched-SLIC-Superpixels)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(camshiftdemo src/camshiftdemo.cpp)
add_executable(linemod src/linemod.cpp)
add_executable(virtual_camera_mono src/virtual_camera_mono.cpp)
add_executable(point_pose_extractor src/point_pose_extractor.cpp)
add_executable(white_balance_converter src/white_balance_converter.cpp)
add_executable(hough_lines src/hough_lines.cpp)
add_executable(rectangle_detector src/rectangle_detector.cpp)
add_executable(calc_flow src/calc_flow.cpp)
add_executable(color_histogram_sliding_matcher src/color_histogram_sliding_matcher.cpp)
add_library(oriented_gradient src/oriented_gradient.cpp)
add_executable(oriented_gradient_node src/oriented_gradient_node.cpp)
add_executable(sliding_window_object_detector_trainer_node  src/sliding_window_object_detector_trainer_node.cpp)

file(GLOB jsk_perception_node_scripts ${PROJECT_SOURCE_DIR}/node_scripts/*)
foreach(_node_script ${jsk_perception_node_scripts})
  get_filename_component(_node_name ${_node_script} NAME_WE)
  get_filename_component(_basename ${_node_script} NAME)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/test/${_node_name}.test)
    list(APPEND jsk_perception_nodes_without_test node_scripts/${_basename})
  endif()
endforeach()

macro(jsk_perception_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/test/${_single_nodelet_exec_name}.test)
    list(APPEND jsk_perception_nodes_without_test ${_nodelet_cpp})
  endif()
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_perception_nodelet_sources jsk_perception_nodelet_executables)
endmacro()
jsk_perception_nodelet(src/bounding_box_to_rect.cpp "jsk_perception/BoundingBoxToRect" "bounding_box_to_rect")
jsk_perception_nodelet(src/rect_array_actual_size_filter.cpp "jsk_perception/RectArrayActualSizeFilter" "rect_array_actual_size_filter")
jsk_perception_nodelet(src/rect_array_to_density_image.cpp "jsk_perception/RectArrayToDensityImage" "rect_array_to_density_image")
jsk_perception_nodelet(src/polygon_array_color_likelihood.cpp "jsk_perception/PolygonArrayColorLikelihood" "polygon_array_color_likelihood")
jsk_perception_nodelet(src/polygon_array_color_histogram.cpp "jsk_perception/PolygonArrayColorHistogram" "polygon_array_color_histogram")
jsk_perception_nodelet(src/edge_detector.cpp "jsk_perception/EdgeDetector" "edge_detector")
jsk_perception_nodelet(src/sparse_image_encoder.cpp "jsk_perception/SparseImageEncoder" "sparse_image_encoder")
jsk_perception_nodelet(src/sparse_image_decoder.cpp "jsk_perception/SparseImageDecoder" "sparse_image_decoder")
jsk_perception_nodelet(src/color_histogram.cpp "jsk_perception/ColorHistogram" "color_histogram")
jsk_perception_nodelet(src/background_substraction_nodelet.cpp "jsk_perception/BackgroundSubstraction" "background_substraction")
jsk_perception_nodelet(src/hough_circles.cpp "jsk_perception/HoughCircleDetector" "hough_circles")
jsk_perception_nodelet(src/fisheye_to_panorama.cpp "jsk_perception/FisheyeToPanorama" "fisheye_to_panorama")
jsk_perception_nodelet(src/grabcut_nodelet.cpp "jsk_perception/GrabCut" "grabcut")
jsk_perception_nodelet(src/slic_superpixels.cpp "jsk_perception/SLICSuperPixels" "slic_super_pixels")
jsk_perception_nodelet(src/rgb_decomposer.cpp "jsk_perception/RGBDecomposer" "rgb_decomposer")
jsk_perception_nodelet(src/hsv_decomposer.cpp "jsk_perception/HSVDecomposer" "hsv_decomposer")
jsk_perception_nodelet(src/lab_decomposer.cpp "jsk_perception/LabDecomposer" "lab_decomposer")
jsk_perception_nodelet(src/ycc_decomposer.cpp "jsk_perception/YCCDecomposer" "ycc_decomposer")
jsk_perception_nodelet(src/contour_finder.cpp "jsk_perception/ContourFinder" "contour_finder")
jsk_perception_nodelet(src/snake_segmentation.cpp "jsk_perception/SnakeSegmentation" "snake_segmentation")
jsk_perception_nodelet(src/colorize_labels.cpp "jsk_perception/ColorizeLabels" "colorize_labels")
jsk_perception_nodelet(src/colorize_float_image.cpp "jsk_perception/ColorizeFloatImage" "colorize_float_image")
jsk_perception_nodelet(src/overlay_image_color_on_mono.cpp "jsk_perception/OverlayImageColorOnMono" "overlay_image_color_on_mono")
jsk_perception_nodelet(src/grid_label.cpp "jsk_perception/GridLabel" "grid_label")
jsk_perception_nodelet(src/color_histogram_label_match.cpp "jsk_perception/ColorHistogramLabelMatch" "color_histogram_label_match")
jsk_perception_nodelet(src/apply_mask_image.cpp "jsk_perception/ApplyMaskImage" "apply_mask_image")
jsk_perception_nodelet(src/unapply_mask_image.cpp "jsk_perception/UnapplyMaskImage" "unapply_mask_image")
jsk_perception_nodelet(src/single_channel_histogram.cpp "jsk_perception/SingleChannelHistogram" "single_channel_histogram")
jsk_perception_nodelet(src/blob_detector.cpp "jsk_perception/BlobDetector" "blob_detector")
jsk_perception_nodelet(src/add_mask_image.cpp "jsk_perception/AddMaskImage" "add_mask_image")
jsk_perception_nodelet(src/multiply_mask_image.cpp "jsk_perception/MultiplyMaskImage" "multiply_mask_image")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/ErodeMaskImage" "erode_mask_image")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/DilateMaskImage" "dilate_mask_image")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/Opening" "opening")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/Closing" "closing")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/MorphologicalGradient" "morphological_gradient")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/TopHat" "top_hat")
jsk_perception_nodelet(src/morphological_operator.cpp "jsk_perception/BlackHat" "black_hat")
jsk_perception_nodelet(src/label_to_mask_image.cpp "jsk_perception/LabelToMaskImage" "label_to_mask_image")
jsk_perception_nodelet(src/rect_to_mask_image.cpp "jsk_perception/RectToMaskImage" "rect_to_mask_image")
jsk_perception_nodelet(src/mask_image_to_roi.cpp "jsk_perception/MaskImageToROI" "mask_image_to_roi")
jsk_perception_nodelet(src/mask_image_to_rect.cpp "jsk_perception/MaskImageToRect" "mask_image_to_rect")
jsk_perception_nodelet(src/bounding_rect_mask_image.cpp "jsk_perception/BoundingRectMaskImage" "bounding_rect_mask_image")
jsk_perception_nodelet(src/bounding_object_mask_image.cpp "jsk_perception/BoundingObjectMaskImage" "bounding_object_mask_image")
jsk_perception_nodelet(src/roi_to_rect.cpp "jsk_perception/ROIToRect" "roi_to_rect")
jsk_perception_nodelet(src/roi_to_mask_image.cpp "jsk_perception/ROIToMaskImage" "roi_to_mask_image")
jsk_perception_nodelet(src/polygon_to_mask_image.cpp "jsk_perception/PolygonToMaskImage" "polygon_to_mask_image")
jsk_perception_nodelet(src/polygon_array_to_label_image.cpp "jsk_perception/PolygonArrayToLabelImage" "polygon_array_to_label_image")
jsk_perception_nodelet(src/rect_to_roi.cpp "jsk_perception/RectToROI" "rect_to_roi")
jsk_perception_nodelet(src/convex_hull_mask_image.cpp "jsk_perception/ConvexHullMaskImage" "convex_hull_mask_image")
jsk_perception_nodelet(src/mask_image_generator.cpp "jsk_perception/MaskImageGenerator" "mask_image_generator")
jsk_perception_nodelet(src/project_image_point.cpp "jsk_perception/ProjectImagePoint" "project_image_point")
jsk_perception_nodelet(src/sliding_window_object_detector.cpp "jsk_perception/SlidingWindowObjectDetector" "sliding_window_object_detector")
jsk_perception_nodelet(src/gaussian_blur.cpp "jsk_perception/GaussianBlur" "gaussian_blur")
jsk_perception_nodelet(src/kmeans.cpp "jsk_perception/KMeans" "kmeans")
if(${OpenCV_VERSION} VERSION_GREATER "2.9.9")  # >= 3.0.0
  jsk_perception_nodelet(src/bing.cpp "jsk_perception/Bing" "bing")
endif()
jsk_perception_nodelet(src/tabletop_color_difference_likelihood.cpp "jsk_perception/TabletopColorDifferenceLikelihood" "tabletop_color_difference_likelihood")
if(OPENMP_FOUND)
  jsk_perception_nodelet(src/saliency_map_generator_node.cpp "jsk_perception/SaliencyMapGenerator" "saliency_map_generator")
  jsk_perception_nodelet(src/skeletonization_nodelet.cpp "jsk_perception/Skeletonization" "skeletonization")
endif()

# robot_self_filter is released on version greater than indigo
find_package(robot_self_filter QUIET)
if(NOT robot_self_filter_FOUND)
  # check existence of robot_self_filter package because robot_self_filter is not installed by apt on version less than indigo
  find_package(PkgConfig)
  pkg_check_modules(robot_self_filter robot_self_filter QUIET)
endif()

if(robot_self_filter_FOUND)
  message(AUTHOR_WARNING "robot_self_filter is found.")
  jsk_perception_nodelet(src/robot_to_mask_image.cpp "jsk_perception/RobotToMaskImage" "robot_to_mask_image")
endif(robot_self_filter_FOUND)

# compiling jsk_perception library for nodelet
add_library(${PROJECT_NAME} SHARED ${jsk_perception_nodelet_sources}
  ${CMAKE_CURRENT_BINARY_DIR}/build/patched-SLIC-Superpixels/slic.cpp
  src/histogram_of_oriented_gradients.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${robot_self_filter_LIBRARIES} yaml-cpp)
if(robot_self_filter_FOUND)
  include_directories(${robot_self_filter_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${robot_self_filter_LIBRARIES})
endif(robot_self_filter_FOUND)
if(${OpenCV_VERSION} VERSION_GREATER "2.4.8")
  # This condition is to avoid changing current behavior
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)


target_link_libraries(camshiftdemo             ${catkin_LIBRARIES})
target_link_libraries(linemod                  ${catkin_LIBRARIES})
target_link_libraries(virtual_camera_mono      ${catkin_LIBRARIES})
target_link_libraries(point_pose_extractor     ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(edge_detector            ${catkin_LIBRARIES})
target_link_libraries(white_balance_converter  ${catkin_LIBRARIES})
target_link_libraries(hough_lines              ${catkin_LIBRARIES})
target_link_libraries(rectangle_detector       ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(calc_flow                ${catkin_LIBRARIES})
target_link_libraries(color_histogram_sliding_matcher  ${catkin_LIBRARIES})
target_link_libraries(oriented_gradient_node   ${catkin_LIBRARIES} ${Boost_LIBRARIES} oriented_gradient)
target_link_libraries(oriented_gradient   ${catkin_LIBRARIES} ${Boost_LIBRARIES} oriented_gradient)
target_link_libraries(sliding_window_object_detector_trainer_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(camshiftdemo             ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(virtual_camera_mono      ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(point_pose_extractor     ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp libsiftfast)
add_dependencies(white_balance_converter  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(hough_lines              ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(rectangle_detector       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(color_histogram_sliding_matcher       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)


add_custom_command(
 OUTPUT ${PROJECT_SOURCE_DIR}/launch/eusmodel_detection_elevator-panels-eng8.launch
 ${PROJECT_SOURCE_DIR}/launch/eusmodel_detection_elevator-panels-eng2.launch
 DEPENDS ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_template_gen.l
 COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} roseus ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_template_gen.l
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_command(
 OUTPUT  ${PROJECT_SOURCE_DIR}/sample/rimokon-pose.launch ${PROJECT_SOURCE_DIR}/sample/milktea-box.launch
 DEPENDS ${PROJECT_SOURCE_DIR}/sample/pose_detector_auto_gen_sample.l
 COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} roseus ./pose_detector_auto_gen_sample.l
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/sample)

add_custom_target(eusmodel_template ALL DEPENDS
  ${PROJECT_SOURCE_DIR}/sample/rimokon-pose.launch ${PROJECT_SOURCE_DIR}/sample/milktea-box.launch
  ${PROJECT_SOURCE_DIR}/launch/eusmodel_detection_elevator-panels-eng8.launch
  ${PROJECT_SOURCE_DIR}/launch/eusmodel_detection_elevator-panels-eng2.launch)

# ------------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------------

install(TARGETS camshiftdemo virtual_camera_mono point_pose_extractor white_balance_converter hough_lines rectangle_detector calc_flow color_histogram_sliding_matcher oriented_gradient oriented_gradient_node ${PROJECT_NAME}
  ${jsk_perception_nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY node_scripts/ scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(DIRECTORY sample launch euslisp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES jsk_perception_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ------------------------------------------------------------------------------------
# Test
# ------------------------------------------------------------------------------------
message(AUTHOR_WARNING "Node/Nodelets without rostest: ${${PROJECT_NAME}_nodes_without_test}")

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  # install test data by a script
  add_custom_target(install_test_data COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)
  if(TARGET tests)
    add_dependencies(tests install_test_data)
  endif()
  set(ROSLINT_CPP_OPTS "--filter=-whitespace/parens,-whitespace/braces")
  roslint_cpp()
  roslint_python()
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # roslaunch_add_file_check in hydro is too strict
    macro(jsk_perception_add_rostest rostest_file)
      add_rostest(${rostest_file})
      roslaunch_add_file_check(${rostest_file})
    endmacro()
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    jsk_perception_add_rostest(test/add_mask_image.test)
    jsk_perception_add_rostest(test/apply_mask_image.test)
    jsk_perception_add_rostest(test/blob_detector.test)
    jsk_perception_add_rostest(test/bof_histogram_extractor.test)
    jsk_perception_add_rostest(test/bounding_object_mask_image.test)
    jsk_perception_add_rostest(test/bounding_rect_mask_image.test)
    jsk_perception_add_rostest(test/color_histogram.test)
    jsk_perception_add_rostest(test/colorize_labels.test)
    jsk_perception_add_rostest(test/colorize_float_image.test)
    jsk_perception_add_rostest(test/draw_rect_array.test)
    jsk_perception_add_rostest(test/extract_image_channel.test)
    jsk_perception_add_rostest(test/image_cluster_indices_decomposer.test)
    jsk_perception_add_rostest(test/image_publisher.test)
    jsk_perception_add_rostest(test/image_to_label.test)
    jsk_perception_add_rostest(test/mask_image_to_label.test)
    jsk_perception_add_rostest(test/kmeans.test)
    jsk_perception_add_rostest(test/label_to_mask_image.test)
    jsk_perception_add_rostest(test/rect_array_to_image_marker.test)
    jsk_perception_add_rostest(test/convex_hull_mask_image.test)
    jsk_perception_add_rostest(test/overlay_image_color_on_mono.test)
    jsk_perception_add_rostest(test/selective_search.test)
    jsk_perception_add_rostest(test/slic_super_pixels.test)
    jsk_perception_add_rostest(test/sklearn_classifier.test)
    jsk_perception_add_rostest(test/split_fore_background.test)
    jsk_perception_add_rostest(test/tile_image.test)
    jsk_perception_add_rostest(test/label_image_decomposer.test)
    # FIXME: Does not work on Travis/Jenkins
    # jsk_perception_add_rostest(test/fcn_object_segmentation.test)
    # jsk_perception_add_rostest(test/vgg16_object_recognition.test install_trained_data)
    if(TARGET bing)
      jsk_perception_add_rostest(test/bing.test)
    endif()
  endif()
  add_rostest(test/sparse_image.test)
endif()
