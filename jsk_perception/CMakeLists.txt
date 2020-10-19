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
endif()

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  image_view2
  jsk_data
  jsk_recognition_msgs
  jsk_recognition_utils
  jsk_topic_tools
  message_generation
  mk
  nodelet
  opencv_apps
  pcl_ros
  posedetection_msgs
  roscpp
  rospack
  sensor_msgs
  std_msgs
  tf)

find_package(roseus)

if(NOT $ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  catkin_python_setup()
endif()

find_package(Boost REQUIRED COMPONENTS filesystem system)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

find_package(Eigen3)
if(EIGEN3_FOUND)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
else()
  # Fallback to cmake_modules
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include(FindPkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
if(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
endif()

pkg_check_modules(libcmt REQUIRED libcmt)

# robot_self_filter is released on version greater than indigo
find_package(robot_self_filter QUIET)
if(NOT robot_self_filter_FOUND)
  # check existence of robot_self_filter package because robot_self_filter is not installed by apt on version less than indigo
  find_package(PkgConfig)
  pkg_check_modules(robot_self_filter robot_self_filter QUIET)
endif()

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

# download and install learning datasets
add_custom_target(${PROJECT_NAME}_install_learning_datasets ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_learning_datasets.py)
# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

# ------------------------------------------------------------------------------------
# Catkin setup
# ------------------------------------------------------------------------------------

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/ConcaveHullMaskImage.cfg
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
  cfg/matchtemplate.cfg
  cfg/point_pose_extractor.cfg
  cfg/RectangleDetector.cfg
  cfg/ColorHistogram.cfg
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
  cfg/FlowVelocityThresholding.cfg
  cfg/FilterMaskImageWithSize.cfg
  cfg/FastRCNN.cfg
  cfg/DrawRects.cfg
  cfg/FacePoseEstimation.cfg
  cfg/LabelImageDecomposer.cfg
  cfg/SSDObjectDetector.cfg
  cfg/MaskRCNNInstanceSegmentation.cfg
  cfg/VirtualCameraMono.cfg
)

message(WARNING "DEPRECATION WARNING: srv files in jsk_pcl_ros are deprecated."
        " Please use those in jsk_recognition_msgs.")
# TODO(wkentaro): Remove deprecated srv files.
add_service_files(FILES
  EuclideanSegment.srv
  NonMaximumSuppression.srv
  SetTemplate.srv
  WhiteBalancePoints.srv
  WhiteBalance.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  jsk_recognition_msgs
  jsk_recognition_utils
  message_runtime
  posedetection_msgs
  sensor_msgs
  std_msgs
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} oriented_gradient
)


# ------------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------------
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/slic/slic.cpp)
  message(WARNING "slic/slic.cpp is not exists, download this")
  execute_process(COMMAND git submodule init slic   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  execute_process(COMMAND git submodule update slic WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/deep_sort/deep_sort/README.md)
  message(WARNING "node_scripts/deep_sort/deep_sort is not exists, download this")
  execute_process(COMMAND git submodule init deep_sort   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/deep_sort)
  execute_process(COMMAND git submodule update deep_sort WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/deep_sort)
endif()

# Build nodelet executables
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} slic ${libcmt_INCLUDE_DIRS})
if(robot_self_filter_FOUND)
  include_directories(${robot_self_filter_INCLUDE_DIRS})
endif()
link_directories(${libcmt_LIBRARY_DIRS})

file(GLOB ${PROJECT_NAME}_node_scripts ${PROJECT_SOURCE_DIR}/node_scripts/*)
foreach(_node_script ${${PROJECT_NAME}_node_scripts})
  get_filename_component(_node_name ${_node_script} NAME_WE)
  get_filename_component(_basename ${_node_script} NAME)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/test/${_node_name}.test)
    list(APPEND ${PROJECT_NAME}_nodes_without_test node_scripts/${_basename})
  endif()
endforeach()

macro(jsk_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/test/${_single_nodelet_exec_name}.test")
    list(APPEND ${PROJECT_NAME}_nodes_without_test ${_nodelet_cpp})
  endif()
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

jsk_add_nodelet(src/filter_mask_image_with_size.cpp "jsk_perception/FilterMaskImageWithSize" "filter_mask_image_with_size")
jsk_add_nodelet(src/flow_velocity_thresholding.cpp "jsk_perception/FlowVelocityThresholding" "flow_velocity_thresholding")
jsk_add_nodelet(src/bounding_box_to_rect.cpp "jsk_perception/BoundingBoxToRect" "bounding_box_to_rect")
jsk_add_nodelet(src/rect_array_actual_size_filter.cpp "jsk_perception/RectArrayActualSizeFilter" "rect_array_actual_size_filter")
jsk_add_nodelet(src/rect_array_to_density_image.cpp "jsk_perception/RectArrayToDensityImage" "rect_array_to_density_image")
jsk_add_nodelet(src/polygon_array_color_likelihood.cpp "jsk_perception/PolygonArrayColorLikelihood" "polygon_array_color_likelihood")
jsk_add_nodelet(src/polygon_array_color_histogram.cpp "jsk_perception/PolygonArrayColorHistogram" "polygon_array_color_histogram")
jsk_add_nodelet(src/sparse_image_encoder.cpp "jsk_perception/SparseImageEncoder" "sparse_image_encoder")
jsk_add_nodelet(src/sparse_image_decoder.cpp "jsk_perception/SparseImageDecoder" "sparse_image_decoder")
jsk_add_nodelet(src/color_histogram.cpp "jsk_perception/ColorHistogram" "color_histogram")
jsk_add_nodelet(src/background_substraction_nodelet.cpp "jsk_perception/BackgroundSubstraction" "background_substraction")
jsk_add_nodelet(src/fisheye_to_panorama.cpp "jsk_perception/FisheyeToPanorama" "fisheye_to_panorama")
jsk_add_nodelet(src/grabcut_nodelet.cpp "jsk_perception/GrabCut" "grabcut")
jsk_add_nodelet(src/slic_superpixels.cpp "jsk_perception/SLICSuperPixels" "slic_super_pixels")
jsk_add_nodelet(src/rgb_decomposer.cpp "jsk_perception/RGBDecomposer" "rgb_decomposer")
jsk_add_nodelet(src/hsv_decomposer.cpp "jsk_perception/HSVDecomposer" "hsv_decomposer")
jsk_add_nodelet(src/lab_decomposer.cpp "jsk_perception/LabDecomposer" "lab_decomposer")
jsk_add_nodelet(src/ycc_decomposer.cpp "jsk_perception/YCCDecomposer" "ycc_decomposer")
jsk_add_nodelet(src/contour_finder.cpp "jsk_perception/ContourFinder" "contour_finder")
if("${OpenCV_VERSION}" VERSION_LESS "3")  # CvSnakeImage has been removed in opencv3
  jsk_add_nodelet(src/snake_segmentation.cpp "jsk_perception/SnakeSegmentation" "snake_segmentation")
endif()
jsk_add_nodelet(src/colorize_labels.cpp "jsk_perception/ColorizeLabels" "colorize_labels")
jsk_add_nodelet(src/colorize_float_image.cpp "jsk_perception/ColorizeFloatImage" "colorize_float_image")
jsk_add_nodelet(src/overlay_image_color_on_mono.cpp "jsk_perception/OverlayImageColorOnMono" "overlay_image_color_on_mono")
jsk_add_nodelet(src/grid_label.cpp "jsk_perception/GridLabel" "grid_label")
jsk_add_nodelet(src/color_histogram_label_match.cpp "jsk_perception/ColorHistogramLabelMatch" "color_histogram_label_match")
jsk_add_nodelet(src/apply_mask_image.cpp "jsk_perception/ApplyMaskImage" "apply_mask_image")
jsk_add_nodelet(src/unapply_mask_image.cpp "jsk_perception/UnapplyMaskImage" "unapply_mask_image")
jsk_add_nodelet(src/single_channel_histogram.cpp "jsk_perception/SingleChannelHistogram" "single_channel_histogram")
jsk_add_nodelet(src/blob_detector.cpp "jsk_perception/BlobDetector" "blob_detector")
jsk_add_nodelet(src/add_mask_image.cpp "jsk_perception/AddMaskImage" "add_mask_image")
jsk_add_nodelet(src/subtract_mask_image.cpp "jsk_perception/SubtractMaskImage" "subtract_mask_image")
jsk_add_nodelet(src/multiply_mask_image.cpp "jsk_perception/MultiplyMaskImage" "multiply_mask_image")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/ErodeMaskImage" "erode_mask_image")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/DilateMaskImage" "dilate_mask_image")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/Opening" "opening")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/Closing" "closing")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/MorphologicalGradient" "morphological_gradient")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/TopHat" "top_hat")
jsk_add_nodelet(src/morphological_operator.cpp "jsk_perception/BlackHat" "black_hat")
jsk_add_nodelet(src/label_to_mask_image.cpp "jsk_perception/LabelToMaskImage" "label_to_mask_image")
jsk_add_nodelet(src/rect_to_mask_image.cpp "jsk_perception/RectToMaskImage" "rect_to_mask_image")
jsk_add_nodelet(src/mask_image_to_roi.cpp "jsk_perception/MaskImageToROI" "mask_image_to_roi")
jsk_add_nodelet(src/mask_image_to_rect.cpp "jsk_perception/MaskImageToRect" "mask_image_to_rect")
jsk_add_nodelet(src/bounding_rect_mask_image.cpp "jsk_perception/BoundingRectMaskImage" "bounding_rect_mask_image")
jsk_add_nodelet(src/bounding_object_mask_image.cpp "jsk_perception/BoundingObjectMaskImage" "bounding_object_mask_image")
jsk_add_nodelet(src/roi_to_rect.cpp "jsk_perception/ROIToRect" "roi_to_rect")
jsk_add_nodelet(src/roi_to_mask_image.cpp "jsk_perception/ROIToMaskImage" "roi_to_mask_image")
jsk_add_nodelet(src/polygon_to_mask_image.cpp "jsk_perception/PolygonToMaskImage" "polygon_to_mask_image")
jsk_add_nodelet(src/polygon_array_to_label_image.cpp "jsk_perception/PolygonArrayToLabelImage" "polygon_array_to_label_image")
jsk_add_nodelet(src/rect_to_roi.cpp "jsk_perception/RectToROI" "rect_to_roi")
jsk_add_nodelet(src/concave_hull_mask_image.cpp "jsk_perception/ConcaveHullMaskImage" "concave_hull_mask_image")
jsk_add_nodelet(src/convex_hull_mask_image.cpp "jsk_perception/ConvexHullMaskImage" "convex_hull_mask_image")
jsk_add_nodelet(src/mask_image_generator.cpp "jsk_perception/MaskImageGenerator" "mask_image_generator")
jsk_add_nodelet(src/project_image_point.cpp "jsk_perception/ProjectImagePoint" "project_image_point")
jsk_add_nodelet(src/sliding_window_object_detector.cpp "jsk_perception/SlidingWindowObjectDetector" "sliding_window_object_detector")
jsk_add_nodelet(src/gaussian_blur.cpp "jsk_perception/GaussianBlur" "gaussian_blur")
jsk_add_nodelet(src/kmeans.cpp "jsk_perception/KMeans" "kmeans")
jsk_add_nodelet(src/draw_rects.cpp "jsk_perception/DrawRects" "draw_rects")
if("${OpenCV_VERSION}" VERSION_GREATER "2.9.9")  # >= 3.0.0
  jsk_add_nodelet(src/bing.cpp "jsk_perception/Bing" "bing")
endif()
jsk_add_nodelet(src/tabletop_color_difference_likelihood.cpp "jsk_perception/TabletopColorDifferenceLikelihood" "tabletop_color_difference_likelihood")
if(OPENMP_FOUND)
  jsk_add_nodelet(src/saliency_map_generator_node.cpp "jsk_perception/SaliencyMapGenerator" "saliency_map_generator")
  jsk_add_nodelet(src/skeletonization_nodelet.cpp "jsk_perception/Skeletonization" "skeletonization")
endif()
if("$ENV{ROS_DISTRO}" STRGREATER "hydro") # >= indigo
  jsk_add_nodelet(src/consensus_tracking.cpp "jsk_perception/ConsensusTracking" "consensus_tracking")
endif()

if(robot_self_filter_FOUND)
  jsk_add_nodelet(src/robot_to_mask_image.cpp "jsk_perception/RobotToMaskImage" "robot_to_mask_image")
endif()

# compiling jsk_perception library for nodelet
add_library(${PROJECT_NAME} SHARED ${jsk_perception_nodelet_sources}
  slic/slic.cpp
  src/histogram_of_oriented_gradients.cpp
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${libcmt_LIBRARIES} yaml-cpp)
if(robot_self_filter_FOUND)
  target_link_libraries(${PROJECT_NAME} ${robot_self_filter_LIBRARIES})
endif()
if("${OpenCV_VERSION}" VERSION_GREATER "2.4.8")
  # This condition is to avoid changing current behavior
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

# Build node executables

macro(jsk_add_node _node_cpp _node_exec_name)
  cmake_parse_arguments(ARG "" "" "DEPENDS;LIBRARIES" ${ARGN})
  add_executable(${_node_exec_name} ${_node_cpp})
  target_link_libraries(${_node_exec_name} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ARG_LIBRARIES})
  add_dependencies(${_node_exec_name} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp ${ARG_DEPENDS})
  list(APPEND ${PROJECT_NAME}_node_executables ${_node_exec_name})
endmacro()

add_library(oriented_gradient src/oriented_gradient.cpp)
target_link_libraries(oriented_gradient ${catkin_LIBRARIES} ${Boost_LIBRARIES})

jsk_add_node(src/camshiftdemo.cpp camshiftdemo)
jsk_add_node(src/linemod.cpp linemod)
jsk_add_node(src/virtual_camera_mono.cpp virtual_camera_mono)
jsk_add_node(src/point_pose_extractor.cpp point_pose_extractor)
jsk_add_node(src/white_balance_converter.cpp white_balance_converter)
jsk_add_node(src/rectangle_detector.cpp rectangle_detector)
jsk_add_node(src/calc_flow.cpp calc_flow)
jsk_add_node(src/color_histogram_sliding_matcher.cpp color_histogram_sliding_matcher)
jsk_add_node(src/oriented_gradient_node.cpp oriented_gradient_node
             DEPENDS oriented_gradient LIBRARIES oriented_gradient)
jsk_add_node(src/sliding_window_object_detector_trainer_node.cpp sliding_window_object_detector_trainer_node
             DEPENDS ${PROJECT_NAME} LIBRARIES ${PROJECT_NAME})

message(STATUS "check if ROSEUS found : ${roseus_FOUND}")
if(roseus_FOUND)
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

# install only template/ direcotry exists, because this is auto-generated directory
install(DIRECTORY template
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

else(roseus_FOUND)
  message(WARNING "could not find 'roseus' so skip eusmodel_template_gen.l and pose_detector_auto_gen_sample.l")
endif(roseus_FOUND)

# ------------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------------

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
install(TARGETS
  # Libraries
  ${PROJECT_NAME}
  oriented_gradient
  # Node executables
  ${${PROJECT_NAME}_node_executables}
  # Nodelet executables
  ${${PROJECT_NAME}_nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config euslisp launch node_scripts plugins sample test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
install(DIRECTORY scripts
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
 )
else()
install(PROGRAMS scripts/setup_opencv_learning_data.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/
)

file(GLOB SCRIPT_PROGRAMS scripts/*.py)
catkin_install_python(
  PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/
)
endif()


# ------------------------------------------------------------------------------------
# Test
# ------------------------------------------------------------------------------------
message(AUTHOR_WARNING "Node/Nodelets without rostest: ${${PROJECT_NAME}_nodes_without_test}")

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  # install test data by a script
  add_custom_target(${PROJECT_NAME}_install_test_data COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)
  if(TARGET tests)
    add_dependencies(tests ${PROJECT_NAME}_install_test_data)
  endif()
  set(ROSLINT_CPP_OPTS "--filter=-whitespace/parens,-whitespace/braces")
  roslint_cpp()
  roslint_python()
  # declare rostest
  macro(jsk_add_rostest rostest_file)
    add_rostest(${rostest_file})
    roslaunch_add_file_check(${rostest_file})
  endmacro()
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # roslaunch_add_file_check in hydro is too strict
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    jsk_add_rostest(test/add_mask_image.test)
    jsk_add_rostest(test/subtract_mask_image.test)
    jsk_add_rostest(test/alexnet_object_recognition.test)
    jsk_add_rostest(test/apply_context_to_label_probability.test)
    jsk_add_rostest(test/apply_mask_image.test)
    jsk_add_rostest(test/background_subtraction.test)
    jsk_add_rostest(test/binpack_rect_array.test)
    jsk_add_rostest(test/blob_detector.test)
    jsk_add_rostest(test/bof_histogram_extractor.test)
    jsk_add_rostest(test/bounding_object_mask_image.test)
    jsk_add_rostest(test/bounding_rect_mask_image.test)
    jsk_add_rostest(test/calc_flow.test)
    jsk_add_rostest(test/color_histogram.test)
    jsk_add_rostest(test/color_histogram_label_match.test)
    jsk_add_rostest(test/colorize_float_image.test)
    jsk_add_rostest(test/colorize_labels.test)
    jsk_add_rostest(test/concave_hull_mask_image.test)
    jsk_add_rostest(test/consensus_tracking.test)
    jsk_add_rostest(test/convex_hull_mask_image.test)
    if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
      jsk_add_rostest(test/deep_sort_tracker.test)
    endif()
    jsk_add_rostest(test/draw_rects.test)
    jsk_add_rostest(test/draw_classification_result.test)
    jsk_add_rostest(test/extract_image_channel.test)
    jsk_add_rostest(test/face_pose_estimation.test)
    # fast_rcnn.test is unstable.
    # So disabled in https://github.com/jsk-ros-pkg/jsk_recognition/pull/2440
    # jsk_add_rostest(test/fast_rcnn.test)
    # fcn_depth_prediction.test is unstable.
    # See https://github.com/jsk-ros-pkg/jsk_recognition/pull/2244
    # jsk_add_rostest(test/fcn_depth_prediction.test)
    jsk_add_rostest(test/fcn_object_segmentation.test)
    add_rostest(test/fcn_object_segmentation.test ARGS use_mask:=true)
    jsk_add_rostest(test/filter_mask_image_with_size.test)
    jsk_add_rostest(test/fisheye_ray.test)
    jsk_add_rostest(test/fisheye_to_panorama.test)
    jsk_add_rostest(test/flow_velocity_thresholding.test)
    jsk_add_rostest(test/gaussian_blur.test)
    jsk_add_rostest(test/grabcut.test)
    jsk_add_rostest(test/hsv_decomposer.test)
    jsk_add_rostest(test/image_cluster_indices_decomposer.test)
    jsk_add_rostest(test/image_publisher.test)
    jsk_add_rostest(test/image_time_diff.test)
    jsk_add_rostest(test/image_to_label.test)
    jsk_add_rostest(test/kalman_filtered_objectdetection_marker.test)
    jsk_add_rostest(test/kmeans.test)
    jsk_add_rostest(test/lab_decomposer.test)
    jsk_add_rostest(test/label_image_classifier.test)
    jsk_add_rostest(test/label_image_decomposer.test)
    jsk_add_rostest(test/label_to_mask_image.test)
    jsk_add_rostest(test/mask_image_to_label.test)
    jsk_add_rostest(test/mask_image_to_roi.test)
    # mask_rcnn_instance_segmentation.test requires GPU.
    # jsk_add_rostest(test/mask_rcnn_instance_segmentation.test)
    jsk_add_rostest(test/matchtemplate.test)
    jsk_add_rostest(test/morphological_operator.test)
    jsk_add_rostest(test/non_maximum_suppression.test)
    jsk_add_rostest(test/overlay_image_color_on_mono.test)
    jsk_add_rostest(test/people_pose_estimation_2d.test)
    jsk_add_rostest(test/human_mesh_recovery.test)
    jsk_add_rostest(test/pointit.test)
    jsk_add_rostest(test/polygon_array_color_histogram.test)
    jsk_add_rostest(test/polygon_array_color_likelihood.test)
    jsk_add_rostest(test/polygon_array_to_label_image.test)
    jsk_add_rostest(test/polygon_to_mask_image.test)
    jsk_add_rostest(test/probability_image_classifier.test)
    jsk_add_rostest(test/project_image_point.test)
    jsk_add_rostest(test/rect_array_to_image_marker.test)
    jsk_add_rostest(test/rect_to_mask_image.test)
    jsk_add_rostest(test/rect_to_roi.test)
    jsk_add_rostest(test/regional_feature_based_object_recognition.test)
    jsk_add_rostest(test/rgb_decomposer.test)
    jsk_add_rostest(test/robot_to_mask_image.test)
    jsk_add_rostest(test/roi_to_mask_image.test)
    jsk_add_rostest(test/roi_to_rect.test)
    jsk_add_rostest(test/saliency_map_generator.test)
    jsk_add_rostest(test/selective_search.test)
    jsk_add_rostest(test/single_channel_histogram.test)
    jsk_add_rostest(test/skeletonization.test)
    jsk_add_rostest(test/sklearn_classifier.test)
    jsk_add_rostest(test/slic_super_pixels.test)
    jsk_add_rostest(test/sliding_window_object_detector.test)
    if(TARGET JSK_NODELET_${PROJECT_NAME}_snake_segmentation)
      jsk_add_rostest(test/snake_segmentation.test)
    endif()
    jsk_add_rostest(test/solidity_rag_merge.test)
    jsk_add_rostest(test/speak_when_label_found.test)
    jsk_add_rostest(test/split_fore_background.test)
    jsk_add_rostest(test/tabletop_color_difference_likelihood.test)
    jsk_add_rostest(test/tile_image.test)
    jsk_add_rostest(test/unapply_mask_image.test)
    jsk_add_rostest(test/unwrap_histogram_with_range_array.test)
    jsk_add_rostest(test/vgg16_object_recognition.test)
    jsk_add_rostest(test/virtual_camera_mono.test)
    jsk_add_rostest(test/ssd_object_detector.test)
    jsk_add_rostest(test/ycc_decomposer.test)
    if(TARGET JSK_NODELET_${PROJECT_NAME}_bing)
      jsk_add_rostest(test/bing.test)
    endif()
    if("$ENV{ROS_DISTRO}" STRLESS "jade")
      # Wait for upstream: ml_classifiers is not yet released in jade, kinetic, ...
      # (https://github.com/jsk-ros-pkg/jsk_recognition/pull/2385#issuecomment-451349430)
      jsk_add_rostest(test/random_forest_server.test)
    endif()
  endif()
  jsk_add_rostest(test/sparse_image.test)
endif()
