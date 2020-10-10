^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imagesift
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.15 (2020-10-10)
-------------------

1.2.14 (2020-10-09)
-------------------

1.2.13 (2020-10-08)
-------------------

1.2.12 (2020-10-03)
-------------------

1.2.11 (2020-10-01)
-------------------
* Fix for  noetic / 20.04 (`#2507 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2507>`_)

  * imagesift/src/imagesift.cpp fix for opencv4
  * upgrade package.xml to format=3, migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration

* fix generate_readme.py and update readme (`#2442 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2442>`_)
* Contributors: Kei Okada, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi

1.2.10 (2019-03-27)
-------------------

1.2.9 (2019-02-23)
------------------

1.2.8 (2019-02-22)
------------------
* fix arm build regressions, closes `#2396 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2396>`_ (`#2397 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2397>`_ )
* Contributors: Kei Okada

1.2.7 (2019-02-14)
------------------

1.2.6 (2018-11-02)
------------------
* Install 'sample' and 'test' dir into CATKIN_PACKAGE_SHARE_DESTINATION (`#2345 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2345>`_)
* fix for jsk-ros-pkg/jsk_common/pull/1586 (`#2311 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2311>`_)
  * use lib\_ instaed of  for add_library to avoid conflict
  * add version_gte for jsk_topic_tools
  * imagesift: fix for jsk-ros-pkg/jsk_common/pull/1586
* Refactor CMakeLists.txt and package.xml of jsk_perception (`#2279 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2279>`_)
* Contributors: Fuki Furuta, Kei Okada, Kentaro Wada

1.2.5 (2018-04-09)
------------------

1.2.4 (2018-01-12)
------------------

1.2.3 (2017-11-23)
------------------
* imagesift: fix onInitPostProcess and poke (`#2203 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2203>`_)
* Contributors: Yuki Furuta

1.2.2 (2017-07-23)
------------------

1.2.1 (2017-07-15)
------------------

1.2.0 (2017-07-15)
------------------

1.1.3 (2017-07-07)
------------------

1.1.2 (2017-06-16)
------------------
* Generate README by script
* Contributors: Kentaro Wada

1.1.1 (2017-03-04)
------------------

1.1.0 (2017-02-09)
------------------

1.0.4 (2017-02-09)
------------------

1.0.3 (2017-02-08)
------------------

1.0.2 (2017-01-12)
------------------

1.0.1 (2016-12-13)
------------------

1.0.0 (2016-12-12)
------------------

0.3.29 (2016-10-30)
-------------------

0.3.28 (2016-10-29)
-------------------

0.3.27 (2016-10-29)
-------------------

0.3.26 (2016-10-27)
-------------------

0.3.25 (2016-09-16)
-------------------

0.3.24 (2016-09-15)
-------------------

0.3.23 (2016-09-14)
-------------------

0.3.22 (2016-09-13)
-------------------
* remove imagesift/CATKIN_IGNORE which is wrongly commited on #1628
* Compile jsk_data from source in released testing
* Compile jsk_data from source in released testing
* jsk_tools/test_topic_published.py won't work on hydro
* Add test for imagesift
* Make imagesift stop depending on jsk_perception
* Install imagesift executable correctly (#1621)
* Contributors: Kei Okada, Kentaro Wada

0.3.21 (2016-04-15)
-------------------

0.3.20 (2016-04-14)
-------------------

0.3.19 (2016-03-22)
-------------------

0.3.18 (2016-03-21)
-------------------

0.3.17 (2016-03-20)
-------------------
* [imagesift] Set ros node name same to the executable name
* [imagesift] Add ORB detector
* Contributors: Ryohei Ueda

0.3.16 (2016-02-11)
-------------------

0.3.15 (2016-02-09)
-------------------
* [imagesift] Use ros::WallTime to measure computation time
* Contributors: Ryohei Ueda

0.3.14 (2016-02-04)
-------------------

0.3.13 (2015-12-19)
-------------------

0.3.12 (2015-12-19)
-------------------

0.3.11 (2015-12-18)
-------------------

0.3.10 (2015-12-17)
-------------------
* [imagesift] INFO -> DEBUG to remove verbose output
* Contributors: Kentaro Wada

0.3.9 (2015-12-14)
------------------

0.3.8 (2015-12-08)
------------------
* Use ccache if installed to make it fast to generate object file
* Contributors: Kentaro Wada

0.3.7 (2015-11-19)
------------------
* [imagesift] Test nodelet and topic /ImageFeature0D also
* [imagesift] Add rostest for imagesift
* [imagesift] Validation of input image shape
* [imagesift] Python lib to wrap siftfastpy
* Contributors: Kentaro Wada

0.3.6 (2015-09-11)
------------------
* [imagesift] Add doc
* [imagesift] Add comments on sample launch
* Contributors: Kentaro Wada

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
* Add cmake_modules
* Contributors: Kei Okada

0.2.14 (2015-08-13)
-------------------
* CMakeLists.txt, imagefeatures.cpp.in: a lot of detector is not available on opencv3
* [imagesift] modify mask region when mask region has no region
* [imagesift] Add sample of imagesift
* [imagesift] Use super class nodehandler
* [imagesift] Make imagesift as nodelet
* Contributors: Kei Okada, Kentaro Wada, Hitoshi Kamada

0.2.13 (2015-06-11)
-------------------
* [imagesift] Add header file of imagesift
* [imagesift] Add Feature0D as output
* Contributors: Kentaro Wada

0.2.12 (2015-05-04)
-------------------

0.2.11 (2015-04-13)
-------------------

0.2.10 (2015-04-09)
-------------------

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

0.2.4 (2015-03-08)
------------------

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
* [imagesift] Refactor codes:
  * uniformed variable naming convention
  * uniformed function naming convention
  * soft tabs
  * no space in if parens
* [imagesift] Better support of masking image:
  1) Use jsk_perception::boundingRectOfMaskImage to compute ROI
  2) support mask image in imagesift.cpp to make better performance
* Contributors: Ryohei Ueda

0.1.33 (2015-01-24)
-------------------
* speed up image sift
* [imagesift] Support mask image in imagefeatures.cpp.in
* Contributors: Ryohei Ueda, Hitoshi Kamada

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------
* [imagesift] Add warning message if size of mask and image are different
* [imagesift] support mask image
* [imagesift] Add warning message if size of mask and image are different
* [imagesift] support mask image
* [imagesift] Fix order of subscription and advertisation

0.1.30 (2014-12-24)
-------------------

0.1.29 (2014-12-24)
-------------------

0.1.28 (2014-12-17)
-------------------

0.1.27 (2014-12-09)
-------------------

0.1.26 (2014-11-23)
-------------------

0.1.25 (2014-11-21)
-------------------

0.1.24 (2014-11-15)
-------------------
* use ifdef not if for OPENCV_NON_FREE
* use OPENCV_NON_FREE option, 14.04 does not provide nonfree libraries
* Contributors: Kei Okada

0.1.23 (2014-10-09)
-------------------
* added codes to check how long it takes to calc keypoints
* added codes to make executable file that use some feature extraction methods
* Contributors: Yu Ohara

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

0.1.11 (2014-07-08)
-------------------

0.1.10 (2014-07-07)
-------------------

0.1.9 (2014-07-01)
------------------

0.1.8 (2014-06-29)
------------------
* maked configure_file to create imagesurf, imagestar and imagebrisk automatically
* added the programs to use cv_detection
* Contributors: Yu Ohara

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
* Contributors: Kei Okada, rosen
