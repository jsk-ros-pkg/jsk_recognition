^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.19 (2016-03-22)
-------------------
* remove rosbuild from run/build depend
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* Contributors: Kei Okada

0.3.18 (2016-03-21)
-------------------
* jsk_perception/CMakeLists.txt: remove depends to rosbuild
* Contributors: Kei Okada

0.3.17 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_perception] binpack_rect_array.py to enumerate jsk_recognition_msgs/RectArray
* [jsk_perception] Add selective_search.py
* [jsk_perception] Use timer callback to speed up tile_image with no_sync:=true
* [jsk_perception] Cache concatenated image to speed up
* Contributors: Kei Okada, Ryohei Ueda

0.3.16 (2016-02-11)
-------------------
* Merge pull request `#1531 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1531>`_ from k-okada/sed_package_xml
  .travis.yml: sed package.xml to use opencv3
* remove image_view2 from find_package(catkin)
* [jsk_perception/CMakeLists.txt] call one of find_package or pkg_check_modules for robot_self_filter.
* [jsk_perception] Set queue_size=1 for tile_image.py
* [jsk_perception] Fix variable names in edge_detector.cpp
* [jsk_perception] Publish result after initialization
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda

0.3.15 (2016-02-09)
-------------------
* U and V has strange library options; https://github.com/ros/rosdistro/pull/10436#issuecomment-180763393
* [jsk_perception] Do not subscribe camera info in calc_flow
* [jsk_perception] Add more 2d feature samples
* Fix label probabilities output message
  Modified:
  - jsk_perception/node_scripts/sklearn_classifier.py
* Add queue_size option for bof_histogram_extractor
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.14 (2016-02-04)
-------------------
* Merge pull request #1513 from garaemon/bounding-box-to-rect-array
  [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* Add ~queue_size option for synchronization
  Modified:
  - jsk_perception/include/jsk_perception/apply_mask_image.h
  - jsk_perception/src/apply_mask_image.cpp
* [jsk_perception/ApplyMask] Add option to clip mask image
  Modified:
  - jsk_perception/include/jsk_perception/apply_mask_image.h
  - jsk_perception/src/apply_mask_image.cpp
* [jsk_perception/tile_image.py] Add ~no_sync parameter to disable
  synchronization of input topics.
* [jsk_perception] Skip for empty sift features
  Modified:
  - jsk_perception/node_scripts/bof_histogram_extractor.py
* [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* [jsk_perception] [kalman-filtered-objectdetection-marker.l] fix code
* added default num_threads\_ value and modified readme.md
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_recognition into saliency_map_generator
  Conflicts:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Except index error on SolidityRagMerge
  Modified:
  - jsk_perception/node_scripts/solidity_rag_merge.py
* parallelized main loop
* [jsk_perception/bof_histogram_extractor.py] Skip if only background image
* [jsk_perception] Skip empty image
* [jsk_perception] Publish info in sample launch file
  Modified:
  - jsk_perception/sample/publish_fixed_images.launch
* [jsk_perception] Stop using deprecated PLUGINLIB_DECLARE_CLASS
  Modified:
  - jsk_perception/src/color_histogram.cpp
  - jsk_perception/src/edge_detector.cpp
  - jsk_perception/src/hough_circles.cpp
  - jsk_perception/src/sparse_image_decoder.cpp
  - jsk_perception/src/sparse_image_encoder.cpp
* [jsk_perception] Add solidity_rag_merge
  This is to find image region with high solidity.
  Firstly, I will use this for vacuum gripper's approach point
  decision making.
  Added:
  - jsk_perception/node_scripts/solidity_rag_merge.py
* [jsk_perception] Set header correctly
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* Merge pull request #1457 from wkentaro/fix-unconfigured-cmake-packagexml
  [jsk_perception] Fix unconfigured cmake and manifest
* Merge pull request #1455 from wkentaro/publish-label-fg-bg
  [jsk_perception] Publish label fg/bg decomposed masks
* [jsk_perception] Check ROS_DISTRO for find_package of robot_self_filter
* [jsk_perception] Fix unconfigured cmake and manifest
  Modified:
  - jsk_perception/CMakeLists.txt
  - jsk_perception/package.xml
* [jsk_perception] Keep original encoding and scale to visualize
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] ColorizeLabels info -> debug
  Modified:
  - jsk_perception/src/colorize_labels.cpp
* [jsk_perception] Add roslint_cpp not as rostest
  Modified:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Publish label fg/bg decomposed masks
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* Merge pull request #1398 from wkentaro/roslint-test-for-node-scripts
  [jsk_perception] Run roslint for python code
* [jsk_perception] Visualize label in label_image_decomposer.py
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Read reference color histogram from a yaml file in PolygonArrayColorLikelihood
  to avoid race condition between input topics
  Modified:
  - doc/jsk_perception/nodes/polygon_array_color_likelihood.md
  - jsk_perception/CMakeLists.txt
  - jsk_perception/include/jsk_perception/polygon_array_color_likelihood.h
  - jsk_perception/package.xml
  - jsk_perception/src/polygon_array_color_likelihood.cpp
* [jsk_perception] Keep original resolution if all the input images has
  same shape and add ~draw_input_topic parameter to draw topic name on
  the tiled images
  Modified:
  - jsk_perception/node_scripts/tile_image.py
  - jsk_recognition_utils/python/jsk_recognition_utils/visualize.py
* Merge pull request #1426 from wkentaro/merge-sklearn-to-jsk-perception
  Merge sklearn to jsk_perception
* [jsk_perception] Add basic_2d_features.launch to overview
  effective technique
  Added:
  - jsk_perception/launch/basic_2d_features.launch
* [jsk_perception] Run roslint for python code
* Merge pull request #1438 from wkentaro/image-to-label
  [jsk_perception] Add image_to_label.py
* [jsk_perception] Use StrictVersions instead of ROS_DISTRO
  Modified:
  - jsk_perception/node_scripts/tile_image.py
* [jsk_perception/label_image_decomposer.py] Fix typo
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception/label_image_decomposer.py] Can specify queue_size
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Fix typo
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Fix tile_image.py for hydro.
  1. Disable approximate sync for hydro. it's not supported on hydro
  2. Use PIL.Image.frombytes instead of PIL.Image.fromstring
* [jsk_perception] Add image_to_label.py
  Added:
  - jsk_perception/node_scripts/image_to_label.py
* [jsk_perception] Fix typo in bof_histogram_extractor.py
  Modified:
  - jsk_perception/node_scripts/bof_histogram_extractor.py
* Merge sklearn to jsk_perception
  Modified:
  jsk_pcl_ros/CMakeLists.txt
  jsk_pcl_ros/package.xml
  jsk_perception/package.xml
  Added:
  jsk_perception/node_scripts/random_forest_server.py
  jsk_perception/sample/random_forest_client_sample.py
  jsk_perception/sample/random_forest_sample.launch
  jsk_perception/sample/random_forest_sample_data_x.txt
  jsk_perception/sample/random_forest_sample_data_y.txt
* added param for printing fps to frame
* nodelet for computing image space saliency map
* Contributors: Kamada Hitoshi, Kei Okada, Kentaro Wada, Ryohei Ueda, Krishneel Chaudhary

0.3.13 (2015-12-19)
-------------------

0.3.12 (2015-12-19)
-------------------
* Revert "[jsk_perception] slic as submodule"
* Contributors: Ryohei Ueda

0.3.11 (2015-12-18)
-------------------
* [jsk_perception] slic as submodule
* Contributors: Ryohei Ueda

0.3.10 (2015-12-17)
-------------------
* [jsk_perception] Add utils to save images by request or from bagfile
  I sent PR to upstream:
  - https://github.com/ros-perception/image_pipeline/pull/159
  - https://github.com/ros-perception/image_pipeline/pull/163
  - https://github.com/ros-perception/image_pipeline/pull/164
  Added:
  jsk_perception/node_scripts/extract_images_sync
  jsk_perception/node_scripts/image_saver_sync
  jsk_perception/node_scripts/publish_header
* [jsk_pcl_ros] Check header.frame_id before resolving 3-D spacially
  Modified:
  jsk_pcl_ros/src/multi_plane_extraction_nodelet.cpp
  jsk_perception/src/polygon_array_color_histogram.cpp
  jsk_recognition_utils/include/jsk_recognition_utils/pcl_ros_util.h
  jsk_recognition_utils/src/pcl_ros_util.cpp
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.9 (2015-12-14)
------------------
* [jsk_perception] Test slop with test_topic_published.py
  Depends on https://github.com/jsk-ros-pkg/jsk_common/pull/1254
* [jsk_perception] Specific test name for each test files
* [jsk_perception] test_topic_published.py does not work on hydro travis/jenkins
  Modified:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Warn about segfault with large size image in SlicSuperpixel
  Modified:
  jsk_perception/src/slic_superpixels.cpp
* [jsk_perception] Test slic_super_pixels
* merge origin/master
* use shared_ptr for self_mask instance.
* Merge remote-tracking branch 'origin/master' into add-robot-mask
* [jsk_perception] Clean up duplicated packages in package.xml
* [jsk_perception] Compute polygon likelihood based on color histogram.
* [jsk_perception] Add PolygonArrayColorHistogram
* add sample launch file.
* add robot_to_mask source files.
* Contributors: Kentaro Wada, Masaki Murooka, Ryohei Ueda

0.3.8 (2015-12-08)
------------------
* [jsk_perception] Add CATKIN_ENABLE_TESTING if block
* Use ccache if installed to make it fast to generate object file
* [jsk_perception] Refactor publish_fixed_images.launch and fix test
* [jsk_perception] Test split_fore_background.py
* [jsk_perception] Fix header of split_fore_background
* [jsk_perception] Refactor publish_fixed_images.launch and fix test
* [jsk_perception] Specify encoding by rosparam in image_publisher.py
* [jsk_perception] Refactor image_publisher.py
* [jsk_perception] Fix supported encodings of split_fore_background.py
  It supports both 16UC1 and 32FC1.
* [jsk_perception] Fix supported encodings of split_fore_background.py
  It supports both 16UC1 and 32FC1.
* [jsk_perception] Add warnNoRemap in ``subscribe()``
* [split fore background] add conversion for depth image format 32FC1
* [jsk_perception] Set frame_id by rosparam
* [jsk_perception] Publish mask also in SplitForeBackground
* add applying blur to output image on edge detector
* [jsk_perception] Split FG/BG with local depth max
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Yohei Kakiuchi

0.3.7 (2015-11-19)
------------------
* Use gcc -z defs to check undefined symbols in shared
  objects (jsk_recognitoin_utils, jsk_pcl_ros, jsk_perception).
  build_check.cpp cannot run on the environment using  multiple processes
  because of invoking libjsk_pcl_ros.so link.
* Merge pull request `#1320 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1320>`_ from wkentaro/colorize_labels-with-146-colors
  [jsk_perception] ColorizeLabels support 20->146 labels
* [jsk_perception] ColorizeLabels support 20->146 labels
* [jsk_perception] Call onInitPostProcess() in last of onInit()
* [jsk_perception] Warn no remapping for input topics
* [jsk_perception] Test whether get topic msg
* [jsk_perception] FastRCNN: (new node)
* [jsk_perception] Test label image decomposer async
* [jsk_perception] Rename SimpleClassifier -> ScikitLearnClassifier
* [jsk_perception] Download trained_data for apc recognition sample
* [jsk_perception] Sort build_depend & run_depend
* [jsk_perception] Publish VectorArray in simple_classifier
* [jsk_perception] Publish VectorArray in bof_histogram_extractor
* [jsk_perception] Convert mask to label image
* [jsk_perception] Convert mask to label image
* [jsk_perception] Make connection based and use ClassificationResult.msg
* [jsk_perception] Care about data size when creating bof data
* [jsk_perception] Specify data size when creating bof data
* [jsk_perception] Update BoF object recognition sample
* [jsk_perception] Extract bof histogram with ConnectionBasedTransport
* [jsk_perception] Create bof & bof_hist dataset
* [jsk_perception] Creating sift dataset script
* [jsk_perception] Move ros node scripts/ -> node_scripts/
  Closes `#1239 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1239>`_
* Merge pull request `#1236 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1236>`_ from wkentaro/slop-param
  [jsk_perception] slop as param for label_image_decomposer
* Merge pull request `#1235 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1235>`_ from wkentaro/skip-0-label-image-decomposer
  [jsk_perception] Skip 0 label in label_image_decomposer
* [jsk_perception] slop as param for label_image_decomposer
* [jsk_perception] Skip 0 label in label_image_decomposer
* [jsk_perception] Debug output about params
* [jsk_perception] Add LabelImageDecomposer
* [jsk_perception] Rename tile_images -> tile_image
* [jsk_perception] Use ConnectionBasedTransport and get_tile_image()
* [jsk_perception/point_pose_extractor] Remove pragma message in compiling
  and fix format warning
* add oriented_gradient and oriented_gradient_node to install target and export libraries
* [jsk_perception] Add tile_images.py
* Contributors: Hiroaki Yaguchi, Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.6 (2015-09-11)
------------------

0.3.5 (2015-09-09)
------------------

0.3.4 (2015-09-07)
------------------
* Swap doc soft links (to make 'Edit on GitHub' work)
* ColorizeFloatImage correct image link
  Closes https://github.com/jsk-ros-pkg/jsk_recognition/issues/1165
* Contributors: Kentaro Wada

0.3.3 (2015-09-06)
------------------
* [jsk_perception] README.md -> readthedocs.org
* Revert "[jsk_perception] use sphinx for rosdoc"
  This reverts commit 9e4ba233599b21c6422ec9a45f395b460c53264d.
* [jsk_perception/TabletopColorDifferenceLikelihood] Use geo/polygon.h
  instead of geo_util.h
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.2 (2015-09-05)
------------------
* [jsk_perception] Ignore autogenerated files
* [jsk_perception] Use histograms to compute distance in TabletopColorDifferenceLikelihood
* Contributors: Ryohei Ueda

0.3.1 (2015-09-04)
------------------
* [jsk_pcl_ros, jsk_perception] Fix dependency of jsk_recognition_utils for child packages
  like jsk_rviz_plugins
* Contributors: Ryohei Ueda

0.3.0 (2015-09-04)
------------------
* [jsk_perception/CMakeLists.txt] set ROS_PACKAGE_PATH before run roseus using package://
* [jsk_recognition_utils] Introduce new package jsk_recognition_utils in order to use utility libraries defined in jsk_pcl_ros in jsk_perception
* Contributors: Kei Okada, Ryohei Ueda

0.2.18 (2015-09-04)
-------------------
* [jsk_perception] Do not specify sexp from cmake, just write in file
* [jsk_perception] Add .gitignore about auto-generated files
* [jsk_perception] Add template directory to run eusmodel_template_gen.l correctly
* [jsk_perception] Add PolygonArrayToLabelImage nodelet
* [jsk_perception] Move matchtemplate.py from src to scripts
* [jsk_perception] Move eusmodel_template_gen.l location from src to euslisp
* [jsk_perception] Do not download trained data in compilation time and
  add script to donload them
* [jsk_perception] use sphinx for rosdoc
* Revert "[jsk_perception] Add rosdoc.yaml to overwrite default file_patterns"
* [package.xml] Updatae Author
* [jsk_perception] use README.md as mainpage.doc
* [jsk_perception] Add rosdoc.yaml to overwrite default file_patterns
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.2.17 (2015-08-21)
-------------------

0.2.16 (2015-08-19)
-------------------
* [CMakeLists.txt] we can not use rospack within cmake process
* Contributors: Kei Okada

0.2.15 (2015-08-18)
-------------------
* Merge pull request `#1058 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1058>`_ from garaemon/uncomment-generate-template
  Uncomment generate template
* [jsk_perception] Add executable flag to eusmodel_template_gen.l
* [jsk_perception] uncomment generate template
* Contributors: JSK-PR2, Ryohei Ueda

0.2.14 (2015-08-13)
-------------------
* [jsk_perception] pub posewithcovariancestamped
* [jsk_perception] Add nodelet ColorizeFloatImage to colorize generic float image
* sliding_window_object_detector : opencv3 has different API for cv::ml::SVM
* src/virtual_camera_mono: use cv.hpp and opencv2 code for cv::getPerspectiveTransform
* src/snake_segmentation: snake (legacy.hpp) is disabled on opencv3
* src/point_pose_extractor: use cv.hpp
* linemode is moved to opencv_contrib, disabled for now (only for opencv3)
* src/calc_flow.cpp: use cv.hpp instead of cv.h
* background_substraction: cv::BackgroundSubtractorMOG2 is abstract type for opencv3
* CMakeLists.txt: depends on cv_bridge, not opencv (jsk_perception)
* [jsk_perception] Update readme
* [jsk_perception] Add simple_classifier*
* [jsk_perception] Scripts for bof and its hist extractor
* do not convert image encode in kmeans and gaussian_blur
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Hitoshi Kamada, Masaki Murooka

0.2.13 (2015-06-11)
-------------------
* [jsk_perception] Use dynamic_reconfigure in ImageTimeDiff
* [jsk_perception] Update image_time_diff to use hue/saturation
* [jsk_perception] Add Kmeans section to README
* [jek_perception] Add kmeans
* [jsk_perception] Add GaussignBlur section to README
* [jsk_perception] Add gaussian_blur
* [jsk_perception] Update README.md for squashing dilate/erode
* [jsk_perception] Squash erode/dilate to morphological_operator
* [jsk_perception] Update README.md for morphological operators
* [jsk_perception] Add advanced morphological transformations
* [jsk_perception] Use isBGR/isRGB/isBGRA/isRGBA in ApplyMaskImage
* [jsk_perception] Add isBGR/isRGB/isBGRA/isRGBA
* [jsk_perception] Use header to synchronize in ImageTimeDiff
* [jsk_perception] Update image_time_diff.py to use ImageDifferenceValue.msg
* [jsk_perception] Update docs of image_time_diff for output
* [jsk_perception] Publish with stamp in image_time_diff
* [jsk_perception/image_publisher] Do not exit program even though no file is found
* uncomment camera_info_cb
* add subscription of image_raw
* Updated Sliding window detector.
  - Removed the trainer
  - Added Bootstraper
* [jsk_perception] Update README for #927
* [jsk_perception] Enable apply_mask convert mask black to transparent
* Changed from reading saved image from directory to RosBag files
* [jsk_perception] Use jsk_topic_tools/log_utils.h for JSK_ROS_INFO,
  JSK_NODELET_INFO and so on
* [jsk_perception] add diff per pixel to ImageTimeDiff
* [jsk_perception] Fix bug in apply_mask in converting BGRA/RGBA input image
* [jsk_perception] remove no need get_param in image_publisher
* [jsk_perception] Enable HSVDecomposer to handle BGRA/RGBA image
* [jsk_perception] Enable ApplyMask handle BGRA/RGBA image
* [jsk_perception] ApplyMask Mono8 encoding to publish mask
* [jsk_perception] Add publish_info param to image_publisher
* [jsk_perception] Add dynamic_reconfigure feature to ImagePublisher
* [jsk_perception] Publish the difference between start and current image
* [jsk_perception][ApplyMaskImage] mask image should be mono8
* Node to for training the classifier for Sliding Window Object Detector
* [jsk_perception] Ignore trained_data directory from git filesystem
* Contributors: Kentaro Wada, Ryohei Ueda, Eisoku Kuroiwa, Krishneel Chaudhary

0.2.12 (2015-05-04)
-------------------
* Revert "[jsk_perception/point_pose_extractor] Use OpenCV's matcher class to estimate mathcing"
* [jsk_perception/point_pose_extractor] Use OpenCV's matcher class to
  estimate mathcing
* [jsk_perception/point_pose_extractor] Add license header
* [jsk_perception] Untabify point_pose_extractor.cpp
* [jsk_perception/point_pose_extractor] Publish PoseStamped from
  point_pose_extractor result
* add ROS_INFO
* [jsk_perception] check if pcam.intrinsicMatrix is valid
* [jsk_perception] Download drill trained data in compiling time
* Removed opencv non-free header directive
  Corrected the nodelet name in CMakeLists.txt
* Corrected the nodelet name in CMakeLists.txt
* Removed opencv non-free header directive
* Nodelet for Edge, Contour Thinning and Nodelet for Sliding window object detector
* [jsk_perception] add Fisheye Rotate parameter
* add upside down option to cfg
* add Fisheye Ray Publisher
* [jsk_perception] Add ProjectImagePoint nodelet to project image local
  coordinates into 3-D point
* [jsk_perception] Update README for fisheye
* [jsk_perception] update Fisheye To Panoarama
* [jsk_perception] Modify typo
* [jsk_perception] Add MaskImageGenerator
* add scale command to shrink the output and make faster
* add cfg
* [jsk_perception] Add fisheye rectify
* [jsk_perception] Add attributeError message to image_publisher.py
* [jsk_perception] Fix README.md about erode/dilate nodelets
* Merge pull request #834 from wkentaro/update-readme-for-pr-811
  [jsk_perception] Update README for histogram max_value of SingleChannelHistogram
* [jsk_perception] Update README for histogram max_value of SingleChannelHistogram
* [jsk_perception] Update README for iterations param of Dilate/ErodeMaskImage
* [jsk_perception] Add iteration param to DilateMaskImage & ErodeMaskImage
* Contributors: Kamada Hitoshi, Kentaro Wada, Ryohei Ueda, Yuto Inagaki, iKrishneel

0.2.11 (2015-04-13)
-------------------
* add encoded points rate
* Contributors: Kamada Hitoshi

0.2.10 (2015-04-09)
-------------------
* [jsk_perception] add Simple Fisheye to Panorama
* [jsk_perception] changed order of dynamic reconfigure
* [jsk_perception] default max value of histogram should be 256 to include 255 pixel
* [jsk_perception] print number of point when encoding sparse image
* [jsk_perception] Publish empty camera info from image_publisher.py
* [jsk_perception] Add sample for ColorHistogramLabelMatch
* [jsk_perception] Add documentation about ColorHistogramLabelMatch
* Contributors: Yuki Furuta, Ryohei Ueda, Yuto Inagaki, Kamada Hitoshi, Kentaro Wada

0.2.9 (2015-03-29)
------------------
* 0.2.8
* Update Changelog
* Contributors: Ryohei Ueda

0.2.8 (2015-03-29)
------------------

0.2.7 (2015-03-26)
------------------

0.2.6 (2015-03-25)
------------------

0.2.5 (2015-03-17)
------------------
* check target cloud data ifnot invalid
* Validate image message without image array (width == 0 and height == 0)
* Enhance: more specific error exception
* Change to avoid SyntaxWarning about not assigning rospy.Publisher argument queue_size
* Change import libs with reasonable order (thirdparty -> ros)
* Contributors: Kentaro Wada, Yu Ohara

0.2.4 (2015-03-08)
------------------
* [jsk_perception] Add simple script to publish image file into ros image
* Fix license: WillowGarage -> JSK Lab
* Contributors: Ryohei Ueda

0.2.3 (2015-02-02)
------------------
* [jsk_pcl_ros, jsk_perception] Move mask image operation to jsk_perception
* Remove rosbuild files
* [jsk_perception] Add ErodeMaskImage nodelet
* [jsk_perception] Add DilateMaskImage
* Contributors: Ryohei Ueda

0.2.2 (2015-01-30)
------------------
* [jsk_perception] add posedetection_msgs
* add image_view2 to depends
* Contributors: Kei Okada

0.2.1 (2015-01-30)
------------------
* add image_view2 to depends

0.2.0 (2015-01-29)
------------------

0.1.34 (2015-01-29)
-------------------
* [jsk_perception, checkerboard_detector] Remove dependency to jsk_pcl_ros
* [jsk_pcl_ros, jsk_perception] Move find_object_on_plane from
  jsk_perception to jsk_pcl_ros to make these packages independent
* [jsk_pcl_ros, jsk_perception] Use jsk_recognition_msgs
* [jsk_pcl_ros, jsk_perception, resized_image_transport] Do not include
  jsk_topic_tools/nodelet.cmake because it is exported by CFG_EXTRAS
* [imagesift] Better support of masking image:
  1) Use jsk_perception::boundingRectOfMaskImage to compute ROI
  2) support mask image in imagesift.cpp to make better performance
* [jsk_perception] Export library
* [jsk_perception] Do not use cv::boundingRect to compute bounding box of
  mask image
* [jsk_perception] install include directory of jsk_perception
* Contributors: Ryohei Ueda

0.1.33 (2015-01-24)
-------------------
* [jsk_perception] FindObjectOnPlane: Find object on plane from 2d binary
  image and 3-d polygon coefficients
* [jsk_perception] Publish convex hull image of mask from ContourFinder
* [jsk_perception] Fix min_area parameter to work in BlobDetector
* [jsk_pcl_ros, jsk_perception] Fix CmakeList for catkin build. Check jsk_topic_tools_SOURCE_PREFIX
* [jsk_perception] Add MultiplyMaskImage
* [jsk_perception] Add ~approximate_sync parameter to toggle
  exact/approximate synchronization
* [jsk_perception] Add UnapplyMaskImage
* [jsk_perception] Add blob image to document
* [jsk_perception] Add BlobDetector
* [jsk_perception] Colorize label 0 as black because label-0 indicates
  masked region
* [jsk_perception] AddMaskImage to add two mask images into one image
* [jsk_perception] Increase label index of SLICSuperPixels to avoid 0. 0
  is planned to be used as 'masked'
* [jsk_perception] Publish result binary image as mono image from ColorHistogramMatch
* [jsk_perception] Extract mask image from coefficients of histogram
  matching in ColorHistogramLabelMatch
* [jsk_perception] Publish result of coefficient calculation as float image
* [jsk_perception] Support mask image in ColorHistogramLabelMatch
* [jsk_perception] Use OpenCV's function to normalize histogram and add
  min and max value of histogram in ColorHistogramLabelMatch
* [jsk_perception] Add ~min_value and ~max_value to SingleChannelHistogram
* [jsk_perception] SingleChannelHistogram to compute histogram of single
  channel image
* [jsk_perception] Add YCrCb decomposer
* [jsk_perception] Add LabDecomposer to decompose BGR/RGB image into Lab
  color space
* [jsk_perception] Use cv::split to split bgr and hsv image into each channel
* [jsk_perception] Fix metrics of ColorHistogramLabelMatch:
  1) correlation
  original value is [-1:1] and 1 is perfect. we apply (1 - x) / 2
  2) chi-squared
  original value is [0:+inf] and 0 is perfect. we apply 1 / (1 + x^2)
  3) intersect
  original value is [0:1] and 1 is perfect. we apply x
  4) bhattacharyya
  original value is [0:1] and 0 is perfect. we apply 1 - x
  5, 6) EMD
  original value is [0:+inf] and 0 is perfect. we apply 1 / (1 + x^2)
* [jsk_perception] Publish more useful debug image from SLICSuperPixels
  and add documentation.
* [jsk_perception] Publish image of interest from ColorHistogram
* [jsk_perception] Implement 6 different method to compute coefficients
  between two histograms
* [jsk_perception] Increase the maximum number of super pixels
* [jsk_perception] Fix ColorHistogram minor bags:
  1. Support rect message out side of image
  2. Use mask image in HSV histogram calculation
* [jsk_perception] Fix HSVDecomposer color space conversion: support RGB8
* [jsk_perception] color matching based on histogram and label information
* [jsk_perception] Add utlity to visualize mask image: ApplyMaskImage
* [jsk_perception] Add GridLabel
* [jsk_perception] Publish hisotgram messages under private namespace
* [jsk_perception] Add simple launch file as sample of superpixels
* [jsk_perception] Utility to colorize labels of segmentation
* [jsk_perception] Fix SLICSuperPixels:
  1) if input image if BGR8
  2) transpose the result of clustering
* [jsk_perception] Publish segmentation result as cv::Mat<int> and use
  patched version of SLIC-SuperPixels to get better performance
* [jsk_perception] Support RGB8 and gray scale color in SLICSuperPixels
* [jsk_perception] Add dynamic_reconfigure interface to SLICSuperPixels
* [jsk_perception] Separate SLICSuperPixels into header and cpp files
* [jsk_perception] Publish result of segmentation of slic superpixels as image
* [jsk_perception] Add snake segmentation
* [jsk_perception] ContourFinder
* [jsk_perception] Support one-channel image in GrabCut
* [jsk_perception] HSVDecomposer to decompose RGB into HSV separate images
* [jsk_perception] Add RGBDecomposer to decompose RGB channels into
  separate images
* Contributors: Ryohei Ueda

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------
* [jsk_perception] Add parameter to select seed policy (definitely
  back/foreground or probably back/foreground) to GrabCut
* adapt attention-clipper for fridge demo
* [jsk_perception] Publish mask image of grabcut result
* [jsk_perception] add GrabCut nodelet
* Remove roseus from build dependency of jsk_perception
* added debug pub

0.1.30 (2014-12-24)
-------------------

0.1.29 (2014-12-24)
-------------------
* added some more parameters for detection
* Contributors: Yu Ohara

0.1.28 (2014-12-17)
-------------------
* added param to set threshold of best_Windoq
* Add dynamic reconfigure to background substraction
* Clean up background substraction codes
* Add background substraction
* Support image mask in ColorHistogram
* Separate header and cpp file of color_hisotgram
* Use jsk_topic_tools::DiagnosticNodelet for color histogram
* Fix coding style of color_histogram
* Fix indent of linemod.cpp
* Add linemod sample
* changed color_histogram_matcher to pub box_array defined in jsk_pcl_ros

0.1.27 (2014-12-09)
-------------------
* added some algolism to get best window
* changed codes to pub center of object
* matchedPointPub by 2dResult of colorhistogram matching
* changed color_histogram_sliding_matcher and added launch to show result
* Contributors: Yu Ohara

0.1.26 (2014-11-23)
-------------------

0.1.25 (2014-11-21)
-------------------
* kalmanfilter
* changed name
* added codes in catkin.cmake
* added cfg
* added color_histogram_mathcer_node

0.1.24 (2014-11-15)
-------------------
* servicecall
* Use intrinsicMatrix instead of projectionMatrix to specify 3x3 matrix(K)
  instead of 4x3 matrix(P)
* remove eigen and add cmake_modules to find_package for indigo
* fix: use projectionMatrix() for indigo
* Add script to setup training assistant for opencv-like dataset
* Add script to check opencv cascade file
* Script to reject positive data for OpenCV training
* renamed only-perception.launch
* calc existance probability
* removed kalmanlib.l from jsk_perception
* add kalman-filter library
* Contributors: Ryohei Ueda, Hitoshi Kamada, Kei Okada, Kamada Hitoshi

0.1.23 (2014-10-09)
-------------------
* Install nodelet executables
* mend spell-miss in launch
* modified program to select which camera_info to sub
* renamed camera_node to uvc_camera_node, and added some options
* modified detection-interface.l
* Contributors: Ryohei Ueda, Kamada, Yu Ohara

0.1.22 (2014-09-24)
-------------------
* Disable ssl when calling git
* Contributors: Ryohei Ueda

0.1.21 (2014-09-20)
-------------------
* Add more diagnostics to OrganizedMultiPlaneSegmentation and fix global
  hook for ConvexHull
* Contributors: Ryohei Ueda

0.1.20 (2014-09-17)
-------------------

0.1.19 (2014-09-15)
-------------------

0.1.18 (2014-09-13)
-------------------
* add git to build_depend of jsk_libfreenect2
* Contributors: Ryohei Ueda

0.1.17 (2014-09-07)
-------------------
* add mk/git to build_depend
* Contributors: Kei Okada

0.1.16 (2014-09-04)
-------------------
* do not use rosrun in the script of jsk_perception/src/eusmodel_template_gen.sh
* Contributors: Ryohei Ueda

0.1.14 (2014-08-01)
-------------------

0.1.13 (2014-07-29)
-------------------

0.1.12 (2014-07-24)
-------------------
* fix to use catkin to link rospack
* Contributors: Kei Okada, Dave Coleman

0.1.11 (2014-07-08)
-------------------
* jsk_perception does not depends on pcl, but depends on eigen and tf
* Contributors: Ryohei Ueda

0.1.10 (2014-07-07)
-------------------
* adding oriented_gradient_node
* add calc_flow program to calc optical flow
* Contributors: Ryohei Ueda, Hiroaki Yaguchi

0.1.9 (2014-07-01)
------------------

0.1.8 (2014-06-29)
------------------
* initialize _img_ptr at first
* convert color image to GRAY
* add nodelet to detect circles based on hough transformation
* add program to compute color histogram (rgb and hsv color space)
* maked configure_file to create imagesurf, imagestar and imagebrisk automatically
* added the programs to use cv_detection
* Contributors: Ryohei Ueda, Yusuke Furuta, Yu Ohara

0.1.7 (2014-05-31)
------------------

0.1.6 (2014-05-30)
------------------

0.1.5 (2014-05-29)
------------------
* add service interface with sensor_msgs/SetCameraInfo to camshiftdemo, not only mouse selection.
* Contributors: Ryohei Ueda

0.1.4 (2014-04-25)
------------------

* add sparse_image program to jsk_percepton
* make edge_detector nodelet class
* Contributors: Ryohei Ueda, Yuki Furuta
* Merge pull request `#47 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/47>`_ from k-okada/add_rosbuild
* Contributors: Kei Okada

0.1.3 (2014-04-12)
------------------

0.1.2 (2014-04-11)
------------------

0.1.1 (2014-04-10)
------------------
* catkinize jsk_perception
* check initialization in check_subscribers function
* change callback function names for avoiding the same name functions
* add edge_detector.launch
* change debug message
* rename type -> atype
* fix minor bug
* change for treating multiple objects in one ObjectDetection.msg
* add test programs
* add rosbuild_link_boost for compile on fuerte/12.04 , see Issue `#224 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/224>`_, thanks tnakaoka
* add rectangle_detector, based on http://opencv-code.com/tutorials/automatic-perspective-correction-for-quadrilateral-objects/
* update hoguh_lines
* use blur before canny
* add image_proc modules from opencv samples
* change error_threshold max 200 -> 2000
* add :detection-topic keyword to (check-detection)
* replace sleep to :ros-wait for making interruptible
* add scripts for speaking english
* speak before sleep
* add to spek we're looking for...
* print out debug info
* turtlebot/ros pdf
* add ros/turtlebot-logo images `#173 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/173>`_
* update japanese speaking
* modify parameter definition. parameter should not be overwritten.
* add option publish-objectdetection-marker
* add slot :diff-rotation in detection_interface.l
* do not create ros::roseus object by load detection_interface.l
* publish tf from sensor frame to detected object pose
* update objectdetection-marker program for new detection_interface
* publish tf and markers, add messages
* print out error value
* fix segfault
* suppor rpy style in relative_pose, status:closed `#139 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/139>`_
* add :target-object keyword to check-detection
* fix : project3dToPixel was removed in groovy
* update to use cv_bridge
* fix for groovy, use cv_bridge not CvBridge
* fix: speak content
* fix: speak-jp
* fix template location
* add microwave detection sample
* add speak-name for speaking japanease object name
* add speak words
* update detction_interface.l for single detection and speak flag
* add solve-tf parameter for not using tf
* add frame_id for coordinates
* add detection_interface.l for using point_pose_extractor
* remove euclidean_cluster,plane_detector and color_extractor from jsk_perception, they are supported in tabletop and pcl apps should go into jsk_pcl_ros
* add max_output
* add opencv2 to rosdep.yaml for compatibility
* update to fit opencv2 electric/fuerte convention
* fix for fuerte see https://code.ros.org/trac/ros/ticket/3955
* add size check
* fix btVector3 -> tf::Vector3
* fix remove define KdTreePtr
* fix style: support ROSPACK_API_V2 (fuerte)
* support ROSPACK_API_V2 (fuerte)
* fix for pcl > 1.3.0, pcl::KdTree -> pcl::search::KdTree, pcl::KdTreeFLANN -> pcl::search::KdTree
* remove explicit dependency to eigen from jsk_perception
* add whilte_balance_param.yaml
* add publish_array for publishing pointsarray
* move posedetectiondb/SetTemplate -> jsk_perception/SetTemplate
* add color_extractor, plane_detector, euclidean_clustering for jsk_perception
* fixed the package name of WhiteBalance.srv
* add eigen to dependency
* add white_balance_converter to jsk_perception
* change msg from face_detector_mono/Rect -> jsk_perception/Rect. I couldn't find set_serch_rect string under jsk-ros-pkg
* node moved from virtual_camera
* check if the matched region does not too big or too small
* add dynamic reconfigure for point_pose_extractor
* split launch for elevator_navigation, to test modules
* fix for oneiric
* fix for users who does not have roseus in their PATH
* ns can't be empty string in launch xml syntax
* commit updates for demo
* added tv-controller with ut logo
* added tv-controller with ut logo
* fixed the size of wrap image, which is calcurated from input (width/height)
* add to write wrapped image
* add error handling and output template file
* add opencv-logo2.png
* add lipton milktea model, auto generated file prefix .launch -> .xml to avoid listed by auto complete
* add sharp rimokon with ist logo
* changed variable name client -> clients
* add sharp tv controller to sample
* add sample for detection launcher generator
* use try to catch assertions
* set Zero as distortionMatrix, because ImageFeature0D.image is rectified
* fixed the box pose in debug image
* changed code for generate SIFT template info
* use projectionMatrix instead of intrinsicMatrix in solvePnP, remove CvBridge -> cv_bridge
* fix to work without roseus path in PATH
* fix relative pose, object coords to texture coords
* update generation script of SIFT pose estimation launcher, relative pose is not correct
* update eusmodel->sift_perception script
* change detection launch generation script to use jsk_perception/point_pose_extractor
* add std namespace appropriately
* update initialize template method
* publish the debug_image of point_pose_extractor
* chnage the output frame id when using only one template
* change threashold for detectiong object
* use /ObjectDetection_agg instead of /ObjectDetection
* add _agg output topic for debug and logging
* add debug message, set lifetime to 1 sec
* add objectdetection-marker.l
* add relative pose parameter to point_pose_extractor.cpp
* change the PutText region
* update sample launch file, point pose extractor do not subscribe input topics when output is not subscribed
* add viewer_window option to disable the OpenCV window
* empty window name to disable window, point_pose_extractor
* move posedetectiondb to jsk_visioncommon
* moved jsk_vision to jsk_visioncommon
* Contributors: Haseru Chen, Kazuto Murai, Youhei Kakiuchi, Yuki Furuta, Kei Okada, Yuto Inagaki, Manabu Saito, Rosen Dinakov, HiroyukiMikita
