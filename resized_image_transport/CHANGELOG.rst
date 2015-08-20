^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package resized_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
