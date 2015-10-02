^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package checkerboard_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2015-09-11)
------------------

0.3.5 (2015-09-09)
------------------

0.3.4 (2015-09-07)
------------------

0.3.3 (2015-09-06)
------------------

0.3.2 (2015-09-05)
------------------

0.3.1 (2015-09-04)
------------------

0.3.0 (2015-09-04)
------------------

0.2.18 (2015-09-04)
-------------------

0.2.17 (2015-08-21)
-------------------

0.2.16 (2015-08-19)
-------------------

0.2.15 (2015-08-18)
-------------------

0.2.14 (2015-08-13)
-------------------
* CMakeLists.txt: depends on cv_bridge, not opencv checkerboard_detector
* src/checkerboard_calibration.cpp: use cv.hpp instead of cv.h
* Contributors: Kei Okada

0.2.13 (2015-06-11)
-------------------
* [checkerboard_detector] Add message_throttle parameter
* [checkerboard_detector] Description in package.xml valid html.
* Contributors: Isaac IY Saito, Ryohei Ueda

0.2.12 (2015-05-04)
-------------------

0.2.11 (2015-04-13)
-------------------

0.2.10 (2015-04-09)
-------------------
* [checkerboard_detector/capture.launch] remove bags in launch
* Contributors: Yu Ohara

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
* [checkerboard_detector] Fill D of camera parameter by zero if ~use_P is true
* Contributors: Ryohei Ueda

0.2.5 (2015-03-17)
------------------
* [checkerboard_detector] add ~use_P to use P instead of K as intrinsic
  parameter for illegal camera info such as multisense
* Contributors: Ryohei Ueda

0.2.4 (2015-03-08)
------------------
* Fix license: WillowGarage -> JSK Lab
* Contributors: Ryohei Ueda

0.2.3 (2015-02-02)
------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

0.2.2 (2015-01-30)
------------------

0.2.1 (2015-01-30)
------------------

0.2.0 (2015-01-29)
------------------

0.1.34 (2015-01-29)
-------------------
* [jsk_perception, checkerboard_detector] Remove dependency to jsk_pcl_ros
* Contributors: Ryohei Ueda

0.1.33 (2015-01-24)
-------------------

0.1.32 (2015-01-12)
-------------------
* [jsk_pcl_ros, checkerboard_detector] Fix offset from checker board
* Contributors: Ryohei Ueda

0.1.31 (2015-01-08)
-------------------
* [checkerboard_detector] Fix compilation warning of
  objectdetection_transform_echo about tf exception
* [checkerboard_detector] Force to ubscribe topic if ~display is True
* [checkerboard_detector] Add modeline for emacs to keep coding style
* something have changed in updatream (maybe cv_bridge), added image_geometry as depends

0.1.30 (2014-12-24)
-------------------

0.1.29 (2014-12-24)
-------------------
* Move multisense specific lines from capture.launch to capture_multisense_training_data.launch
* Added new nodelet to capture training data of stereo camera to
  jsk_pcl_ros and update launch files to capture training data of multisense
* Add launch file to capture training data with two-checker-boarded table
* Add launch file for capture board: publishing center of the capture
  board calculated from two checker board
* Hotfix for mulformed multisense camera_info. Their K and R matrix and
  distirtion parameter is not set
* Add script to estimate position of the camera using two checker boards
* Stabilize color inverted asymetrical circle detection
  1) use cv::bitwise_not to invert color
  2) use cv::CALIB_CB_CLUSTERING when detecting circlesGrid
* Use OpenCV C++ API in checkerboard_detector
* Support color inversion to distinguish white-black circle pattern
  and black-white circle pattern
* Support ciecle and acircle pattern
* Contributors: Ryohei Ueda

0.1.28 (2014-12-17)
-------------------

0.1.27 (2014-12-09)
-------------------
* forget to install objectdetection_tf_publisher.py
* Merge pull request `#457 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/457>`_ from YoheiKakiuchi/update_objectdetection_tf
  update objectdetection_tf_publisher for publishing simple tf
* update objectdetection_tf_publisher for publishing simple tf
* add / to service name
* Contributors: Yohei Kakiuchi, Kei Okada, Yuto Inagaki

0.1.26 (2014-11-23)
-------------------

0.1.25 (2014-11-21)
-------------------

0.1.24 (2014-11-15)
-------------------
* Update depth calibration program.
  1. Fix checkerboard_detector to publish correct corner point
  2. Calibrate depth_image rather than PointCloud
  3. Use matplotlib animation to visualize graph in depth_error_calibration.py
* Publish checker board region as jsk_pcl_ros/PolygonArray
* Publish geometry_msgs/PoseStamped from checkerboard_detector
* Contributors: Ryohei Ueda

0.1.23 (2014-10-09)
-------------------

0.1.22 (2014-09-24)
-------------------

0.1.21 (2014-09-20)
-------------------

0.1.20 (2014-09-17)
-------------------

0.1.19 (2014-09-15)
-------------------

0.1.18 (2014-09-13)
-------------------

0.1.17 (2014-09-07)
-------------------

0.1.16 (2014-09-04)
-------------------

0.1.14 (2014-08-01)
-------------------

0.1.13 (2014-07-29)
-------------------

0.1.12 (2014-07-24)
-------------------
* add two nodelets (DelayPointCloud and DepthImageError) to jsk_pcl_ros
  and publish u/v coordinates of the checkerboard from checkerboard_detector.
  * DepthImageError is just a skelton yet.
  * DelayPointCloud re-publishes pointcloud with specified delay time.
  * publish u/v coordinates from checkerboard_detector.
  * frame_id broadcasted from objectdetection_tf_publisher.py is configurable
* Contributors: Ryohei Ueda

0.1.11 (2014-07-08)
-------------------

0.1.10 (2014-07-07)
-------------------

0.1.9 (2014-07-01)
------------------

0.1.8 (2014-06-29)
------------------

0.1.7 (2014-05-31)
------------------

0.1.6 (2014-05-30)
------------------

0.1.5 (2014-05-29)
------------------

0.1.4 (2014-04-25)
------------------

0.1.3 (2014-04-12)
------------------

0.1.2 (2014-04-11)
------------------

0.1.1 (2014-04-10)
------------------
* install programs
* fix depend package -> rosdep name
* adding rosconsole to its dependency
* add example : update tf position everytime he receves objectdetection msg
* update tf position everytime he receves objectdetection msg
* update objectdetection_tf_publisher by using tf msg directly
* update objectdetection_tf_publisher.py
* add python program for translating the result of checkerboard_detector to tf
* add_dependences to posedetection_msgs_gencpp
* use USE_ROSBUILD for catkin/rosbuild environment
* use ROS_Distributions instead of ROS_DISTRO for electric
* comment out : add catkin.cmake
* add catkin.cmake
* fixed the name bug
* forget to fix checkerboard_calibration [`#154 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/154>`_]
* fix to compile with cv_bridge/cv_bridge, [`#154 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/154>`_]
* enable to set display flag for cvNamedWindow
* add checkerboard_detector_single.launch for single checkerboard detection
* change: If there is no subscriber, node stop subscribing image / camera_info topics (shutdown subscriber)
* fix deperecated message asscessor see http://ros.org/wiki/fuerte/Migration#error:_XXX_has_no_member_named_.27set_YYY_size.27_.28or_.27get_YYY_size.27.29
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* add maxboard param, use when you know how many checkerboards in the environment
* add code for detecting subpix position using geometry of detected points,this code came from checkerboard_pose
* moved jsk_vision to jsk_visioncommon
* moved vision packages to jsk_vision
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* Contributors: nozawa, kazuto, Kei Okada, youhei, rosen, Ryohei Ueda
