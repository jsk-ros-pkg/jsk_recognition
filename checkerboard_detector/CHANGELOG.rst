^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package checkerboard_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Kei Okada, k-okada, kazuto, nozawa, rosen, ueda, youhei
