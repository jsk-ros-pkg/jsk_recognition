^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.10 (2015-04-09)
-------------------
* [jsk_perception] add Simple Fisheye to Panorama
* [jsk_perception] changed order of dynamic reconfigure
* [jsk_perception] default max value of histogram should be 256 to include 255 pixel
* [jsk_perception] print number of point when encoding sparse image
* [jsk_perception] Publish empty camera info from image_publisher.py
* [jsk_perception] Add sample for ColorHistogramLabelMatch
* [jsk_perception] Add documentation about ColorHistogramLabelMatch
* Contributors: Yuki Furuta, Kamada Hitoshi, Kentaro Wada, Ryohei Ueda, Yuto Inagaki

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
* Contributors: Yu Ohara, Kentaro Wada

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
