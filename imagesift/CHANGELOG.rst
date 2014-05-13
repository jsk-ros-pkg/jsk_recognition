^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imagesift
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2014-04-25)
------------------

0.1.3 (2014-04-12)
------------------
* use pkg_check_moduels for libsiftfast, due to https://github.com/jsk-ros-pkg/jsk_common/pull/380
* Contributors: Kei Okada

0.1.2 (2014-04-11)
------------------

0.1.1 (2014-04-10)
------------------
* catkinize imagesift
* catkinize imagesift
* update to use cv_bridge
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* use rosdep opencv2 and pkg-config, as described in the wiki http://www.ros.org/wiki/opencv2
* fix typo for opencv version check
* include nonfree/nonfree.hpp for OpenCV 2.4
* Switch to using the standard vector API: get_data_size() -> data.size()
* moved jsk_vision to jsk_visioncommon
* moved vision packages to jsk_vision
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* Contributors: Kei Okada, k-okada, rosen
