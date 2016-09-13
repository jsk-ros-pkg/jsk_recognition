^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pcl_ros_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.22 (2016-09-13)
-------------------
* [jsk_pcl_ros_utils/delay_point_cloud] Modified using message_filter for delay message
* [jsk_pcl_ros_utils/delay_point_cloud] Modified delay_point's time stampe
* [jsk_pcl_ros_utils/delay_point_cloud] Modified delay_time as dynamic parameter
* [jsk_pcl_ros_utils/delay_point_cloud] Refactor sleep_time -> delay_time
* [jsk_pcl_ros_utils] add test for polygon_array_unwrapper nodelet
* [jsk_pcl_ros_utils] add ~use_likelihood to polygon_array_unwrapper
* Retry at most three times point_indices_to_mask_image.test (#1848)
  To fix error sometimes on Travis.
* Convert cluster indices to point indices with index in rosparam (#1794)
  * Convert cluster indices to point indices with dynamic reconfigure
  * Test ClusterPointIndicesToPointIndices
  * Doc for ClusterPointIndicesToPointIndices
  * Not build cluster_point_indices_to_point_indices on hydro
* Add description about naming rule
* Fix test names in favor to {NODE_NAME}.test
* Negative index is skipped in conversion
* Add test for bounding_box_array_to_bounding_box
* Add sample for bounding_box_array_to_bounding_box
* Convert bounding box array to bounding box
* Fix typo in label_to_cluster_point_indices.h
* Convert point cloud to point indices
* Convert point cloud to mask image in a node
* Convert point indices to mask w/o sync if it's static
* Convert point indices to cluster point indices
  ex)
  - Input Indices: [0, 10, 20]
  - Output Cluster Indices: [[0, 10, 20]]
* [jsk_pcl_ros_utils/PointCloudToPCD] add test and sample launch
* [jsk_pcl_ros_utils/PointCloudToPCD] license modified to JSK
* [jsk_pcl_ros_utils] modify PointCloudToPCD to nodelet and add dynamic_reconfigure
* Stop passing -z flag to ld with clang (#1610)
* Contributors: Kentaro Wada, Shingo Kitagawa, Yuki Furuta, Iori Yanokura

0.3.21 (2016-04-15)
-------------------

0.3.20 (2016-04-14)
-------------------
* [jsk_pcl_ros] add jsk_pcl version of tabletop_object_detector launch/config (`#1585 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1585>`_)
  * [jsk_pcl_ros_utils/jsk_pcl_nodelets.xml] fix: pcl class name typo of CloudOnPlane
  * [jsk_pcl_ros/sample/tabletop_object_detector.launch] add jsk version of tabletop_object_detector
* Contributors: Yuki Furuta

0.3.19 (2016-03-22)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* Contributors: Kei Okada

0.3.18 (2016-03-21)
-------------------

0.3.17 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* Contributors: Kei Okada

0.3.16 (2016-02-11)
-------------------

0.3.15 (2016-02-09)
-------------------

0.3.14 (2016-02-04)
-------------------
* Add ~queue_size option for synchronization
  Modified:
  - jsk_pcl_ros_utils/include/jsk_pcl_ros_utils/point_indices_to_mask_image.h
  - jsk_pcl_ros_utils/src/point_indices_to_mask_image_nodelet.cpp
* Merge pull request #1504 from garaemon/tracking-velocity
  [jsk_pcl_ros] Publish current tracking status (running or idle) from     particle_fitler_tracking.
* [jsk_pcl_ros_utils] Add CloudOnPlane and scripts to visualize them
* [jsk_pcl_ros] Publish current tracking status (running or idle)
  from particle_fitler_tracking.
  And add some scripts to visualize them.
* [jsk_pcl_ros_utils] Use jsk_pcl_utils prefix instead of jsk_pcl to prevent namespace conflict with jsk_pcl nodelets
* [jsk_pcl_ros_utils] Support inliers in plane rejector
  Modified:
  - jsk_pcl_ros_utils/cfg/PlaneRejector.cfg
  - jsk_pcl_ros_utils/include/jsk_pcl_ros_utils/plane_rejector.h
  - jsk_pcl_ros_utils/src/plane_rejector_nodelet.cpp
* [jsk_pcl_ros_utils] Document about LabelToClusterPointIndices
* [jsk_pcl_ros_utils] Add doc symlink
  Added:
  - jsk_pcl_ros_utils/doc
* [jsk_pcl_ros_utils] Add label to cluster point indices
  Modified:
  - jsk_pcl_ros_utils/CMakeLists.txt
  - jsk_pcl_ros_utils/jsk_pcl_nodelets.xml
  Added:
  - jsk_pcl_ros_utils/include/jsk_pcl_ros_utils/label_to_cluster_point_indices.h
  - jsk_pcl_ros_utils/src/label_to_cluster_point_indices_nodelet.cpp
* [jsk_pcl_ros_utils] Remove sklearn from build_depend
  Modified:
  - jsk_pcl_ros_utils/package.xml
  - jsk_pcl_ros_utils/CMakeLists.txt
* [jsk_pcl_ros] Support offset specifying by geometry_msgs/PoseStamped in ICPRegistration
  Modified:
  - doc/index.rst
  - doc/jsk_pcl_ros/nodes/icp_registration.md
  - jsk_pcl_ros/include/jsk_pcl_ros/icp_registration.h
  - jsk_pcl_ros/src/icp_registration_nodelet.cpp
  - jsk_pcl_ros_utils/CMakeLists.txt
  - jsk_pcl_ros_utils/jsk_pcl_nodelets.xml
  Added:
  - doc/jsk_pcl_ros_utils/index.rst
  - doc/jsk_pcl_ros_utils/nodes/pointcloud_relative_form_pose_stamped.md
  - jsk_pcl_ros_utils/include/jsk_pcl_ros_utils/pointcloud_relative_from_pose_stamped.h
  - jsk_pcl_ros_utils/src/pointcloud_relative_from_pose_stamped_nodelet.cpp
* [jsk_pcl_ros -> jsk_pcl_ros_utils] Left migration of PointIndicesToMaskImage
  Modified:
  jsk_pcl_ros/jsk_pcl_nodelets.xml
  jsk_pcl_ros_utils/jsk_pcl_nodelets.xml
* Contributors: Kentaro Wada, Ryohei Ueda, Iori Kumagai

0.3.13 (2015-12-19)
-------------------
* [jsk_pcl_ros_utils] Remove jsk_pcl_ros_base
* Contributors: Ryohei Ueda

0.3.12 (2015-12-19)
-------------------
* update CHANGELOG
* [jsk_pcl_ros_utils] Introduce new package called jsk_pcl_ros_utils
  in order to speed-up compilation of jsk_pcl_ros
* Contributors: Ryohei Ueda

0.3.11 (2015-12-18)
-------------------

0.3.10 (2015-12-17)
-------------------

0.3.9 (2015-12-14)
------------------

0.3.8 (2015-12-08)
------------------

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

0.3.1 (2015-09-04 17:12)
------------------------

0.3.0 (2015-09-04 12:37)
------------------------

0.2.18 (2015-09-04 01:07)
-------------------------

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
