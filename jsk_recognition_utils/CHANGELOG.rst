^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_recognition_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2017-03-04)
------------------

1.1.0 (2017-02-09)
------------------

1.0.4 (2017-02-09)
------------------
* [jsk_recognition_utils] src/geo/segment.cpp: fix argument name of Segment::midpoint. (`#2013 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2013>`_ )
* Contributors: Masaki Murooka

1.0.3 (2017-02-08)
------------------
* [jsk_recognition_utils] add mipoint method to segment class. (`#2009 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2009>`_ )
  * src/edge_depth_refinement_nodelet.cpp
  * src/geo/segment.cpp
   include/jsk_recognition_utils/geo/segment.h
* Evaluate voxel segmentation by IU (`#1993 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1993>`_ )
  * Stop depending on jsk_interactive_marker
  * node_scripts/evaluate_voxel_segmentation_by_gt_box.py
  * Compute box overlap and publish it : intersect-over-union (overlap)  = volume_tp / (volume_fn + volume_fp + volume_tp)
    * test/evaluate_box_segmentation_by_gt_box.test
    * test/evaluate_voxel_segmentation_by_gt_box.test
    * sample/sample_evaluate_box_segmentation_by_gt_box.launch
    * sample/sample_evaluate_voxel_segmentation_by_gt_box.launch
    * scripts/evaluate_box_segmentation_by_gt_box.py
    * scripts/evaluate_voxel_segmentation_by_gt_box.py
  * Move evaluation scripts of box segmentation from jsk_recognition_utils to to jsk_pcl_ros_utils

* Contributors: Kentaro Wada, Masaki Murooka

1.0.2 (2017-01-12)
------------------

1.0.1 (2016-12-13)
------------------

1.0.0 (2016-12-12)
------------------
* Fix fo kinetic  (`#1943 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1943>`_)
  * use std::isnan instead of isnan, knetic compiler requires this

* Contributors: Kei Okada

0.3.29 (2016-10-30)
-------------------

0.3.28 (2016-10-29)
-------------------

0.3.27 (2016-10-29)
-------------------

0.3.26 (2016-10-27)
-------------------
* Stop using deprecated jsk_topic_tools/log_utils.h (`#1933 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1933>`_)
* [heightmap] change type of heightmap to image/32FC2 (`#1886 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1886>`_)
* Prettify the style of rosparam for bbox publisher (`#1885 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1885>`_)
  This shows deprecation warning and does not break the current api.
  (BTW, this code is quite new and I think no one use this other than me.)
* Contributors: Kentaro Wada, Yohei Kakiuchi

0.3.25 (2016-09-16)
-------------------

0.3.24 (2016-09-15)
-------------------

0.3.23 (2016-09-14)
-------------------

0.3.22 (2016-09-13)
-------------------
* Merge pull request #1826 from mmurooka/polyarr-to-poly2
  [jsk_recognition_utils/node_scripts] add polygon_array_to_polygon.py
* [jsk_recognition_utils/node_scripts] add polygon_array_to_polygon.py
* Skip rostest on hydro because of unreleased test tools
* Add test for bounding_box_array_publisher.py
* Add sample for bounding_box_array_publisher.py
* Node to publish bounding box array
* Skip rostest on hydro because of unreleased test tools
* Add test for bounding_box_array_publisher.py
* Add sample for bounding_box_array_publisher.py
* Node to publish bounding box array
* Merge pull request #1809 from wkentaro/feature/pose-array-to-pose
  Convert PoseArray to PoseStamped with a specified index
* Convert PoseArray to PoseStamped with a specified index
* Rename test files in favor to {NODE_NAME}.test
* Add util to convert image 16uc1 to 32fc1
* Merge pull request #1694 from wkentaro/get-numpy-include-dirs
  [jsk_recognition_utils] Set Numpy include directories in cmake to fix error on OS X
* Set Numpy include directories in cmake to fix error on OS X
* Remove color_gategoryXX (use labelcolormap)
* Add label color utility function
* Remove nms.py that is moved to nms.pyx
* Recognize object with VGG16 net
* Rename vgg16 -> vgg16_fast_rcnn
* Cythonize Non-maximum Supression baseline
* Remove dependency on rbgirshick/fast-rcnn
* Support old scipy which does not have face()
* Add static virtual camera
* Copy jsk_perception/image_utils.h to jsk_recognition_utils/cv_utils.h
* Stop passing -z flag to ld with Clang (#1601)
* Contributors: Kei Okada, Kentaro Wada, Masaki Murooka

0.3.21 (2016-04-15)
-------------------

0.3.20 (2016-04-14)
-------------------
* [jsk_recognition_utils] Support Affine3d project function in Plane (`#1576 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1576>`_)
* [jsk_recognition_utils] Add multiple ClusterPointIndices to one (`#1581 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1581>`_)
  * Add multiple ClusterPointIndices to one
  Added:
  - jsk_recognition_utils/node_scripts/add_cluster_indices.py
  * Document for add_cluster_indices.py
* Visualize ClusterPointIndices for image (`#1579 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1579>`_)
* Contributors: Iori Kumagai, Kentaro Wada

0.3.19 (2016-03-22)
-------------------

0.3.18 (2016-03-21)
-------------------

0.3.17 (2016-03-20)
-------------------
* [jsk_perception] Use timer callback to speed up tile_image with no_sync:=true
* [jsk_perception] Cache concatenated image to speed up
* Contributors: Ryohei Ueda

0.3.16 (2016-02-11)
-------------------

0.3.15 (2016-02-09)
-------------------

0.3.14 (2016-02-04)
-------------------
* [jsk_recognition_utils] Tile different size images with centerization
  Modified:
  - jsk_recognition_utils/python/jsk_recognition_utils/visualize.py
* [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* jsk_recognition_utils/CMakeLists.txt: include_directories should have include/ before catkin_INCLUDE_DIRS
* Merge remote-tracking branch 'origin/master' into auto-change-point-type
* [jsk_pcl_ros] Publish current tracking status (running or idle)
  from particle_fitler_tracking.
  And add some scripts to visualize them.
* [jsk_pcl_ros] Automatically detect point type in OctreeVoxelGrid
  Modified:
  - doc/jsk_pcl_ros/nodes/octree_voxel_grid.md
  - jsk_pcl_ros/cfg/OctreeVoxelGrid.cfg
  - jsk_pcl_ros/include/jsk_pcl_ros/octree_voxel_grid.h
  - jsk_pcl_ros/src/octree_voxel_grid_nodelet.cpp
  - jsk_recognition_utils/include/jsk_recognition_utils/pcl_ros_util.h
  - jsk_recognition_utils/src/pcl_ros_util.cpp
* [jsk_recognition_utils] Add SeriesedBoolean::isAllTrueFilled method
  to check all the buffer is filled by true
* [jsk_pcl_ros] Fix WallDurationTimer to publish correct average value
* [jsk_pcl_ros] Publish computation time in icp_registration and torus_finder
  Modified:
  - doc/jsk_pcl_ros/nodes/icp_registration.md
  - doc/jsk_pcl_ros/nodes/torus_f_inder.md
  - jsk_pcl_ros/include/jsk_pcl_ros/icp_registration.h
  - jsk_pcl_ros/include/jsk_pcl_ros/torus_finder.h
  - jsk_pcl_ros/src/icp_registration_nodelet.cpp
  - jsk_pcl_ros/src/torus_finder_nodelet.cpp
  - jsk_recognition_utils/include/jsk_recognition_utils/time_util.h
* [jsk_perception] Keep original resolution if all the input images has
  same shape and add ~draw_input_topic parameter to draw topic name on
  the tiled images
  Modified:
  - jsk_perception/node_scripts/tile_image.py
  - jsk_recognition_utils/python/jsk_recognition_utils/visualize.py
* [jsk_perception] Fix tile_image.py for hydro.
  1. Disable approximate sync for hydro. it's not supported on hydro
  2. Use PIL.Image.frombytes instead of PIL.Image.fromstring
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.13 (2015-12-19)
-------------------

0.3.12 (2015-12-19)
-------------------

0.3.11 (2015-12-18)
-------------------

0.3.10 (2015-12-17)
-------------------
* [jsk_recognition_utils] Fix import error on server caused by matplotlib
* [jsk_pcl_ros] Check header.frame_id before resolving 3-D spacially
  Modified:
  jsk_pcl_ros/src/multi_plane_extraction_nodelet.cpp
  jsk_perception/src/polygon_array_color_histogram.cpp
  jsk_recognition_utils/include/jsk_recognition_utils/pcl_ros_util.h
  jsk_recognition_utils/src/pcl_ros_util.cpp
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.9 (2015-12-14)
------------------
* [jsk_perception] Compute polygon likelihood based on color histogram.
* [jsk_perception] Add PolygonArrayColorHistogram
* [jsk_recognition_utils] Better API to measure and publish computation time
* Contributors: Ryohei Ueda

0.3.8 (2015-12-08)
------------------
* Use ccache if installed to make it fast to generate object file
* [jsk_recognition_utils, jsk_pcl_ros] Measure time to compute
  NormalEstimationOMP and RegionGriwongMultiplePlaneSegmentation.
  Add utility class to measure time: jsk_recognition_utils::WallDurationTimer
* [jsk_recognition_utils] Split fore/background with depth
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.7 (2015-11-19)
------------------
* Use gcc -z defs to check undefined symbols in shared
  objects (jsk_recognitoin_utils, jsk_pcl_ros, jsk_perception).
  build_check.cpp cannot run on the environment using  multiple processes
  because of invoking libjsk_pcl_ros.so link.
* Merge pull request `#1319 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1319>`_ from wkentaro/146-various-rgb-colors
  [jsk_recognition_utils] Add labelToRGB with 146 rgb colors
* Merge pull request `#1324 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1324>`_ from wkentaro/test-tf-listener-singleton
  [jsk_recognition_utils] Test tf_listener_singleton.cpp
* [jsk_recognition_utils] Test rgb_colors.cpp
* [jsk_recognition_utils] Test labelToRGB
* [jsk_recognition_utils] 146 rgb colors
* [jsk_recognition_utils] Test tf_listener_singleton.cpp
* [jsk_recognition_utils] Add labelToRGB
* [jsk_recognition_utils] 146 rgb colors
* [jsk_recognition_utils] Util to decompose descriptors with label
* [jsk_recognition_utils] Test tf::Transformer::lookupTransformation
* [jsk_recognition_utils] Bag of Features as python module
* [jsk_recognition_utils] Handle canvas to get safely plot image
* [jsk_recognition_utils] Add bounding_rect_of_mask
* [jsk_recognition_utils] Add jsk_recognition_utils.get_tile_image()
* [jsk_recognition_utils] Fix laser model
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.6 (2015-09-11)
------------------

0.3.5 (2015-09-09)
------------------

0.3.4 (2015-09-07)
------------------
* Merge pull request `#1168 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1168>`_ from k-okada/add_yaml
  jsk_recognition_utils: forget to add include to install
* jsk_recognition_utils: forget to add include to install
* [jsk_recognition_utils/README] Add link to doxygen documentation
* [jsk_recognition_utils/Line] Add documentation
* Contributors: Kei Okada, Ryohei Ueda

0.3.3 (2015-09-06)
------------------
* [jsk_recognition_utils] Depends on visualization_msgs
* [jsk_recognition_utils] Separate grid_plane.h from geo_util.h
* [jsk_recognition_utils] Separate cylinder.h from geo_util.h
* [jsk_recognition_utils] Separate cube.h from geo_util.h
* [jsk_recognition_utils] Separate convex_polygon.h from geo_util.h
* [jsk_recognition_utils] Separate polygon.h from geo_util.h
* [jsk_recognition_utils] Separate plane.h from geo_util.h
* [jsk_recognition_utils] Separate segment.h from geo_util.h
* [jsk_recognition_utils] Separate line.h from geo_util.h
* Contributors: Ryohei Ueda

0.3.2 (2015-09-05)
------------------
* add yaml-cpp to depends
* Merge pull request `#1151 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1151>`_ from garaemon/use-histograms
  [jsk_perception] Use histograms to compute distance in TabletopColorDifferenceLikelihood
* [jsk_perception] Use histograms to compute distance in TabletopColorDifferenceLikelihood
* Contributors: Kei Okada, Ryohei Ueda

0.3.1 (2015-09-04)
------------------
* Add README.md to jsk_recognition_utils
* Contributors: Ryohei Ueda

0.3.0 (2015-09-04)
------------------
* [jsk_recognition_utils] Introduce new package jsk_recognition_utils
  in order to use utility libraries defined in jsk_pcl_ros in jsk_perception
* Contributors: Ryohei Ueda

0.2.18 (2015-09-04)
-------------------
* [jsk_recognition_utils] Introduce new package jsk_recognition_utils
  in order to use utility libraries defined in jsk_pcl_ros in jsk_perception
* Contributors: Ryohei Ueda

0.2.17 (2015-08-21)
-------------------

0.2.16 (2015-08-19)
-------------------

0.2.15 (2015-08-18)
-------------------

0.2.14 (2015-08-13)
-------------------

0.2.13 (2015-06-11)
-------------------

0.2.12 (2015-05-04)
-------------------

0.2.11 (2015-04-13)
-------------------

0.2.10 (2015-04-09)
-------------------

0.2.9 (2015-03-29)
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

0.2.2 (2015-01-30 19:29)
------------------------

0.2.1 (2015-01-30 00:35)
------------------------

0.2.0 (2015-01-29 12:20)
------------------------

0.1.34 (2015-01-29 11:53)
-------------------------

0.1.33 (2015-01-24)
-------------------

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------

0.1.30 (2014-12-24 16:45)
-------------------------

0.1.29 (2014-12-24 12:43)
-------------------------

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

0.1.15 (2014-08-26)
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
