^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package resized_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

  * resized_image_transport/src/log_polar_nodelet.cpp fix for opencv4

* fix generate_readme.py and update readme (`#2442 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2442>`_)
* Add sample, test and doc (`#2440 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2440>`_)

  * Remove unnecessary dependency duplication and comments in package.xml
  * Add test for LogPolar
  * Add sample for LogPolar

* Contributors: Kei Okada, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi

1.2.10 (2019-03-27)
-------------------

1.2.9 (2019-02-23)
------------------

1.2.8 (2019-02-22)
------------------

1.2.7 (2019-02-14)
------------------
* [resized_image_transport] install resized_image_transport (`#2390 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2390>`_ )
* Contributors: Shingo Kitagawa

1.2.6 (2018-11-02)
------------------
* Call USE_SOURCE_PERMISSIONS before PATTERN(`#2345 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2345>`_)
* [resized_image_transport] Changed diagnostic error level (`#2312 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2312>`_)
  * add version_gte for jsk_topic_tools

* fix for jsk-ros-pkg/jsk_common/pull/1586 (`#2311 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2311>`_)
* Contributors: Yuki Furuta, Yuto Uchimi, Iori Yanokura

1.2.5 (2018-04-09)
------------------

1.2.4 (2018-01-12)
------------------
* Fix uninitialized pointer error in some recognition nodelets (`#2234 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2234>`_)
  * [image_processing] Check validity of vital checker in updateDiagnostic because this callback might be called before initPublishersAndSubscribers is finished
* Contributors: Iori Kumagai

1.2.3 (2017-11-23)
------------------

1.2.2 (2017-07-23)
------------------
* current source code needs jsk_topic_tools >= 2.2.4 (`#2173 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2173>`_)
* Contributors: Kei Okada

1.2.1 (2017-07-15)
------------------

1.2.0 (2017-07-15)
------------------

1.1.3 (2017-07-07)
------------------

1.1.2 (2017-06-16)
------------------
* Remove dependency of resized_image_transport on jsk_perception (`#2075 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2075>`_ )
* Generate README by script (`#2064 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2064>`_ )
* Improve documentation and example of image_resizer (`#2056 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2056>`_ )
  * Add description about rosparam in doc
    - modified:   doc/resized_image_transport/nodes/image_resizer.rst
    - new file:   resized_image_transport/doc
  * Add sample for image_resizer
    - deleted:    resized_image_transport/launch/example.launch
    - new file:   resized_image_transport/launch/sample_image_resizer.launch
    - modified:   resized_image_transport/test/image_resizer.test
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
* Merge branch 'master' into fix_for_kinetic
* remove depends to driver_base
* Contributors: Kei Okada

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
* .travis.yml: add test to check if this works with release repository (#1595)
  * .travis.yml: add test to check if this works with release repository
  * update jsk_travis to 0.4.1
  * resized_image_transport: fix to work with jsk_topic_tools < 2.0.10
  * update jsk_travis 0.4.2
* Add onInitPostProcess for image_resizer (#1590)
  Modified:
  - resized_image_transport/src/image_resizer_nodelet.cpp
* [resized_image_transport] Add test for image_resizer (#1589)
  * Fix deprecated error about advertiseCamera
  Modified:
  - resized_image_transport/src/image_processing_nodelet.cpp
  * Test if image_resizer's output topic is published
  Modified:
  - resized_image_transport/CMakeLists.txt
  Added:
  - resized_image_transport/test/image_resizer.test
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
* remove dynamic_reconfigure.parameter_generator, which is only used for rosbuild
* Contributors: Kei Okada

0.3.16 (2016-02-11)
-------------------

0.3.15 (2016-02-09)
-------------------

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

0.3.9 (2015-12-14)
------------------
* [resized_image_transport] Use jsk_topic_tools::DiagnosticNodelet to omit
  meaningless computation
* Contributors: Ryohei Ueda

0.3.8 (2015-12-08)
------------------
* Use ccache if installed to make it fast to generate object file
* Contributors: Kentaro Wada

0.3.7 (2015-11-19)
------------------

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
* src/log_polar_nodelet.cpp : convert fro milimage to cvmat
* [resized_image_transport] Initialize vital_checkers first
* [resized_image_transport] Add diagnostic information
* Contributors: Kei Okada, Ryohei Ueda

0.2.13 (2015-06-11)
-------------------
* [resized_image_transport] Fix coding style of image_processing_nodelet.cpp
* [resized_image_transport] Do not require synchronized input topics of
  camera info and image unless ~use_camera_subscriber is true
* [resized_image_transport] untabify source code
* [resized_image_transport] Advertise publishers before subscribing topics
* [resized_image_transport] Supress output from image_resizer
* Contributors: Ryohei Ueda

0.2.12 (2015-05-04)
-------------------
* [resized_image_transport] Fix dynamic_reconfigure name in LogPolar
* [resized_image_transport] Pass private nodehandle to dynamic_reconfigure to set handle the name of dynamic_reconfigrue in nodelet correctly
* [resized_image_transport] change from linear to non-linear
* rename to NODELET info and short fix
* [resized_image_transport] image_resizer_nodelet resize rate feedback
* Contributors: Kamada Hitoshi, Ryohei Ueda

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
* [resized_image_transport] Fix ordert to read rosparam and set
  dynamic_reconfigure to use initial value of dynamic_reconfigure correctly
* Contributors: Ryohei Ueda

0.2.4 (2015-03-08)
------------------
* [resized_image_transport] Publish scale information
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
* depending on cv_bridge is recommended, see http://wiki.ros.org/indigo/Migration#OpenCV
* [jsk_pcl_ros, jsk_perception, resized_image_transport] Do not include
  jsk_topic_tools/nodelet.cmake because it is exported by CFG_EXTRAS
* [resized_image_transport] Fix jsk_topic_tools/nodelet.cmake path
* Contributors: Ryohei Ueda, Kei Okada

0.1.33 (2015-01-24)
-------------------
* add parameter to select interpolation method
* Contributors: Yusuke Furuta

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------
* not include image prosessing config
* add log polar sample
* add include directory
* implement resize image processing
* implement log-polar processing
* add base class for processing image
* add sample launch file
* add LogPolar.cfg
* add first sample

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

0.1.23 (2014-10-09)
-------------------
* Install nodelet executables
* Contributors: Ryohei Ueda

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
* Creating publisher before subscribe topics in resized_image_transport
* Supress messages from resized_image_transport
* Contributors: Ryohei Ueda

0.1.17 (2014-09-07)
-------------------

0.1.16 (2014-09-04)
-------------------
* remove static variables from ImageResizer because now it is used as
  nodelet
* add client for resize image
* Contributors: Ryohei Ueda, Yusuke Furuta

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

0.1.7 (2014-05-31)
------------------

0.1.6 (2014-05-30)
------------------
* src/image_resizer.cpp: fix to compile on rosbuild

0.1.5 (2014-05-29)
------------------

0.1.4 (2014-04-25)
------------------

0.1.3 (2014-04-12)
------------------

0.1.2 (2014-04-11)
------------------
* use find_module to check catkin/rosbuild to pass git-buildpackage
* Contributors: Kei Okada

0.1.1 (2014-04-10)
------------------
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: add depend tags
* add depend to driver_base
* add update with message
* simplify example and rename to example.launch
* fix bugs whcn resize paramater is 0, see issue `#252 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/252>`_
* use Kbps not kB, issue `#253 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/253>`_
* updating for catkin
* add option to change fps, rename image_type->image, see Issue 248
* mv resized_imagetransport resized_image_transport
* Contributors: Ryohei Ueda, Kei Okada, Youhei Kakiuchi
