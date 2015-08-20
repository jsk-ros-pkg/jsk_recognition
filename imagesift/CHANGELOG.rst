^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imagesift
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
