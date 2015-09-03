cmake_minimum_required(VERSION 2.8.3)
project(jsk_perception)

find_package(catkin REQUIRED COMPONENTS
  jsk_recognition_msgs
  mk message_generation std_msgs sensor_msgs geometry_msgs cv_bridge
  image_geometry image_transport image_view2 driver_base dynamic_reconfigure cmake_modules
  roscpp nodelet rostest tf rospack
  jsk_topic_tools pcl_ros)

find_package(Boost REQUIRED COMPONENTS filesystem system signals)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
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
  cfg/GaussianBlur.cfg
  cfg/KMeans.cfg
  cfg/ImageTimeDiff.cfg
)

add_service_files(FILES EuclideanSegment.srv  SetTemplate.srv  WhiteBalancePoints.srv  WhiteBalance.srv NonMaximumSuppression.srv)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime jsk_recognition_msgs
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES jsk_perception
)

execute_process(
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR}
  make -f ${PROJECT_SOURCE_DIR}/Makefile.slic
  INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
  MK_DIR=${mk_PREFIX}/share/mk installed
  RESULT_VARIABLE _make_failed)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/build/patched-SLIC-Superpixels)
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

macro(jsk_perception_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_perception_nodelet_sources jsk_perception_nodelet_executables)
endmacro()
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
jsk_perception_nodelet(src/rect_to_mask_image.cpp "jsk_perception/RectToMaskImage" "rect_to_mask_image")
jsk_perception_nodelet(src/mask_image_to_roi.cpp "jsk_perception/MaskImageToROI" "mask_image_to_roi")
jsk_perception_nodelet(src/mask_image_to_rect.cpp "jsk_perception/MaskImageToRect" "mask_image_to_rect")
jsk_perception_nodelet(src/roi_to_rect.cpp "jsk_perception/ROIToRect" "roi_to_rect")
jsk_perception_nodelet(src/roi_to_mask_image.cpp "jsk_perception/ROIToMaskImage" "roi_to_mask_image")
jsk_perception_nodelet(src/polygon_to_mask_image.cpp "jsk_perception/PolygonToMaskImage" "polygon_to_mask_image")
jsk_perception_nodelet(src/polygon_array_to_label_image.cpp "jsk_perception/PolygonArrayToLabelImage" "polygon_array_to_label_image")
jsk_perception_nodelet(src/rect_to_roi.cpp "jsk_perception/RectToROI" "rect_to_roi")
jsk_perception_nodelet(src/mask_image_generator.cpp "jsk_perception/MaskImageGenerator" "mask_image_generator")
jsk_perception_nodelet(src/project_image_point.cpp "jsk_perception/ProjectImagePoint" "project_image_point")
jsk_perception_nodelet(src/skeletonization_nodelet.cpp "jsk_perception/Skeletonization" "skeletonization")
jsk_perception_nodelet(src/sliding_window_object_detector.cpp "jsk_perception/SlidingWindowObjectDetector" "sliding_window_object_detector")
jsk_perception_nodelet(src/gaussian_blur.cpp "jsk_perception/GaussianBlur" "gaussian_blur")
jsk_perception_nodelet(src/kmeans.cpp "jsk_perception/KMeans" "kmeans")

# compiling jsk_perception library for nodelet
add_library(${PROJECT_NAME} SHARED ${jsk_perception_nodelet_sources}
  ${CMAKE_CURRENT_BINARY_DIR}/build/patched-SLIC-Superpixels/slic.cpp
  src/image_utils.cpp
  src/histogram_of_oriented_gradients.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
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
target_link_libraries(sliding_window_object_detector_trainer_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(camshiftdemo             ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(virtual_camera_mono      ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(point_pose_extractor     ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp libsiftfast)
add_dependencies(white_balance_converter  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(hough_lines              ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(rectangle_detector       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(color_histogram_sliding_matcher       ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)


add_custom_command(
 OUTPUT  ${PROJECT_SOURCE_DIR}/template
 DEPENDS ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_template_gen.l
 COMMAND roseus ${PROJECT_SOURCE_DIR}/euslisp/eusmodel_template_gen.l
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_command(
 OUTPUT  ${PROJECT_SOURCE_DIR}/sample/rimokon-pose.launch ${PROJECT_SOURCE_DIR}/sample/milktea-box.launch
 DEPENDS ${PROJECT_SOURCE_DIR}/sample/pose_detector_auto_gen_sample.l
 COMMAND roseus ./pose_detector_auto_gen_sample.l \"(progn\ (generate-launcher)\ (exit))\"
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/sample)

add_custom_target(eusmodel_template ALL DEPENDS ${PROJECT_SOURCE_DIR}/sample/rimokon-pose.launch ${PROJECT_SOURCE_DIR}/sample/milktea-box.launch)

install(TARGETS camshiftdemo virtual_camera_mono point_pose_extractor white_balance_converter hough_lines rectangle_detector calc_flow color_histogram_sliding_matcher ${PROJECT_NAME}
  ${jsk_perception_nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY sample launch euslisp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES jsk_perception_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_rostest(test/sparse_image.test)
