^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pcl_ros_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [jsk_pcl_ros] Add multi euclidean clustering (`#2463 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2463>`_)
* Fix for  noetic / 20.04 (`#2507 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2507>`_)

  * upgrade package.xml to format=3, migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration

* set time-limit and increase retry for jsk_pcl_ros_utils/test/tf_transform_bounding_box_array.test (`#2495 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2495>`_)

  * set time-limit=25 for timeout:30 tests

* set chainer version less than 7.0.0 (`#2485 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2485>`_)

  * Moved bagfile for multi object detection Fixed path of play_rosbag xml [jsk_pcl_ros_utils/install_sample_data.py] Make it multiprocess downloadable

* fix generate_readme.py and update readme (`#2442 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2442>`_)
* MaskImageToDepthConsideredMaskImage: support approximate sync (`#2410 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2410>`_)
* Add sample, test and doc (`#2440 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2440>`_)

  * Add test for DepthImageError
  * Add sample for DepthImageError
  * Add test for PointCloudToSTL
  * Add sample for PointCloudToSTL
  * Fix mesh_resource in output marker msg in PointCloudToSTL
  * Add test for ColorizeDistanceFromPlane
  * Add sample for ColorizeDistanceFromPlane
  * Add test for PoseWithCovarianceStampedToGaussianPointCloud
  * Add sample for PoseWithCovarianceStampedToGaussianPointCloud
  * Add test for PolygonPointsSampler
  * Add sample for PolygonPointsSampler
  * Add test for PolygonFlipper
  * Add sample for PolygonFlipper
  * Add test for PolygonArrayWrapper
  * Add sample for PolygonArrayWrapper
  * Add sample for PolygonArrayUnwrapper
  * Add test for PolygonArrayTransformer
  * Add sample for PolygonArrayTransformer
  * Add test for PlaneConcatenator
  * Add sample for PlaneConcatenator
  * Add test for MarkerArrayVoxelToPointCloud
  * Add sample for MarkerArrayVoxelToPointCloud
  * Add test for PointCloudToClusterPointIndices
  * Add sample for PointCloudToClusterPointIndices
  * Support skip_nan in PointCloudToClusterPointIndices
  * Add test for LabelToClusterPointIndices
  * Add sample for LabelToClusterPointIndices
  * Add test for SphericalPointCloudSimulator
  * Add sample for SphericalPointCloudSimulator
  * Add test for PlanarPointCloudSimulator
  * Move sample for PlanarPointCloudSimulator to jsk_pcl_ros_utils and do not use deprecated node
  * Add test for PointCloudRelativeFromPoseStamped
  * Add sample for PointCloudRelativeFromPoseStamped
  * Support approximate_sync in PointCloudRelativeFromPoseStamped
  * Add test for NormalFlipToFrame
  * Add sample for NormalFlipToFrame
  * Add test for PCDReaderWithPose
  * Add sample for PCDReaderWithPose
  * Add test for TfTransformCloud
  * Add sample for TfTransformCloud
  * Add test for TfTransformBoundingBoxArray
  * Add sample for TfTransformboundingBoxArray
  * Add test for TfTransformBoundingBox
  * Add sample for TfTransformBoundingBox
  * Add test for PolygonArrayFootAngleLikelihood
  * Add sample for PolygonArrayFootAngleLikelihood
  * Add test for PolygonArrayDistanceLikelihood
  * Add sample for PolygonArrayDistanceLikelihood
  * Add test for PolygonArrayAreaLikelihood
  * Add sample for PolygonArrayAreaLikelihood
  * Add test for PolygonArrayAngleLikelihood
  * Add sample for PolygonArrayAngleLikelihood
  * Add test for DelayPointCloud
  * Add sample for DelayPointCloud
  * Add test for ColorizeHeight2DMapping
  * Add sample for ColorizeHeight2DMapping
  * Increase publishing rate of pcd_to_pointcloud in sample_pointcloud_xyz_to_xyzrgb.launch
  * Add missing test for PointCloudXYZToXYZRGB
  * Add test for PointCloudXYZRGBToXYZ
  * Add sample for PointCloudXYZRGBToXYZ
  * Explicitly depend on jsk_rviz_plugins in jsk_pcl_ros_utils/package.xml
  * Add test for cloud_on_plane_info.py
  * Add test for CloudOnPlane
  * Add sample for CloudOnPlane and cloud_on_plane_info.py
  * Support approximate_sync in CloudOnPlane
  * Add test for MaskImageToDepthConsideredMaskImage
  * MaskImageToDepthConsideredMaskImage: support approximate sync

* MaskImageToPointIndices: support multi channel mask image (`#2409 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2409>`_)

  * Enable all test for MaskImageToPointIndices
  * Increase threshold to support JPEG compression as much as possible
  * Use NODELET_ERROR instead of ROS_ERROR
  * Fix access to each element of image
  * Partially disable mask_image_to_point_indices.test
  * Add sample for MaskImageToPointIndices
  * Publish to another topic if ~use_multi_channels is true and ~target_channel == -1
  * Merge branch 'master' into subtract-mask-image
  * MaskImageToPointIndices: support multi channel mask image

* Contributors: Kei Okada, Shingo Kitagawa, Yuki Furuta, Yuto Uchimi, Iory Yanokura

1.2.10 (2019-03-27)
-------------------
* Re-enable pointcloud_to_pcd.test `#2402 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2402>`_)
* [doc] [jsk_pcl_ros_utils] [jsk_pcl_ros] Add documentation (`#2393 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2393>`_)

  * Add test for TransformPointcloudInBoundingBox
  * Add sample for TransformPointcloudInBoundingBox
  * Add test for PlaneReasoner
  * Add sample for PlaneReasoner
  * Add test for PlaneRejcetor
  * Add sample for PlaneRejcetor
  * Add test for PolygonAppender
  * Add sample for PolygonAppender
  * Add test for StaticPolygonArrayPublisher
  * Add sample for StaticPolygonArrayPublisher
  * Add test for NormalConcatenater
  * Add sample for NormalConcatenater

* Contributors: Yuto Uchimi

1.2.9 (2019-02-23)
------------------

1.2.8 (2019-02-22)
------------------

1.2.7 (2019-02-14)
------------------
* [jsk_pcl_ros, jsk_pcl_ros_utils] Use ccache if installed to make it fast to generate object file (`#2342 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2342>`_ )
* Contributors: Iori Yanokura

1.2.6 (2018-11-02)
------------------
* [jsk_pcl_ros_utils/cluster_point_indices_to_point_indices] Concatenate all indices in case of index==-1 (`#2330 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2330>`_)
* [jsk_pcl_ros_utils/package.xml] Add dependencies for compressed_image/depth_image_transport to run sample launch files (`#2341 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2341>`_)
* Install 'sample', 'scripts', 'test' into SHARE_DESTINATION (`#2345 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2345>`_)
* [jsk_perception] Retrain bof data for sklearn==0.2.0 version and modified jsk_pcl_ros/utils's test for kinetic travis (`#2337 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2337>`_)
  * [jsk_pcl_ros_utils] Ignore test for pointcloud_to_pcd.test

* Add --pkg-path option to install_sample_data.py not to use rospack (`#2314 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2314>`_)
  * Close https://github.com/jsk-ros-pkg/jsk_recognition/pull/2303

* fix for jsk-ros-pkg/jsk_common/pull/1586 (`#2311 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2311>`_)
  * to avoid add_custom_target cannot create target install_sample_data because another target with the same name already exists errors

* Use diagnostic nodelet for EuclideanClustering and other nodelets (`#2301 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2301>`_)
  * jsk_pcl_ros: euclidean_clustering: use dianogistc nodelet
    Use DiagnosticNodelet::updateDiagnostic preferrably

* Fix warnings for jsk_pcl_ros_utils (`#2265 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2265>`_)
  * Fix warnings for jsk_pcl_ros_utils
    ```
  CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'pcl' but neither 'pcl_INCLUDE_DIRS' nor
  'pcl_LIBRARIES' is defined.
  Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  CMakeLists.txt:220 (catkin_package)
  CMake Warning (dev) at CMakeLists.txt:214 (add_dependencies):
  Policy CMP0046 is not set: Error on non-existent dependency in
  add_dependencies.  Run "cmake --help-policy CMP0046" for policy details.
  Use the cmake_policy command to set the policy and suppress this warning.
  The dependency target "jsk_pcl_ros_utils_gencpp" of target
  "jsk_pcl_ros_utils" does not exist.
  This warning is for project developers.  Use -Wno-dev to suppress it.
    ```
* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Yuto Uchimi, Iori Yanokura

1.2.5 (2018-04-09)
------------------

1.2.4 (2018-01-12)
------------------
* jsk_pcl_ros_utils: pointcloud_to_mask_image:  add depth image for input (`#2229 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2229>`_)
    jsk_pcl_ros_utils: add depth image for input to pointcloud_to_mask_image
    jsk_pcl_ros_utils: update doc for pointcloud_to_mask_image
* Contributors: Yuki Furuta

1.2.3 (2017-11-23)
------------------
*  [jsk_pcl_ros_utils] polygon_flipper: add option '~use_indices' (`#2189 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2189>`_)
* Contributors: Yuki Furuta

1.2.2 (2017-07-23)
------------------

1.2.1 (2017-07-15)
------------------

1.2.0 (2017-07-15)
------------------

1.1.3 (2017-07-07)
------------------
* Filter invalid centroid in centroid_publisher (`#2150 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2150>`_)
  * Looser timeout for centroid_publisher.test
  * Add sample and test for CentroidPublisher
  * Filter invalid centroid in centroid_publisher

* Capability of specifying background label for LabelToClusterPointIndices (`#2134 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2134>`_)
  * fix bug in label_to_cluster_point_indices_nodelet
  * Capability of specifying background label for LabelToClusterPointIndices

* add ignore_labels in label_to_cluster_point_indices (`#2151 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2151>`_)
  * Fix style of code of LabelToClusterPointIndices

* [jsk_pcl_ros_utils/src] add onInitPostProcess forStaticPolygonArrayPublisher, PolygonArrayTransformer (`#2126 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2126>`_)
  * [jsk_pcl_ros_utils] add onInitPostProcess to static_polygon_array_publisher_nodelet.cpp, polygon_array_transformer_nodelet.cpp

* Contributors: Kanae Kochigami, Kentaro Wada, Shingo Kitagawa

1.1.2 (2017-06-16)
------------------
* [jsk_pcl_ros_utils] add PolygonArrayLikelihoodFilter (`#2054 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2054>`_ )
  * [jsk_pcl_ros_utils] add sample / test for polygon_array_likelihood_filter
  * [jsk_pcl_ros_utils][polygon_array_likelihood_filter] fix
  * [jsk_pcl_ros_utils] add polygon_array_likelihood_filter
    [jsk_pcl_ros_utils] add docs for polygon_array_likelihood_filter
* Add PointCloudXYZRGBToXYZ: (add for testing) (https://github.com/jsk-ros-pkg/jsk_recognition/commit/86b64a27d00d218b68e3d598220cd0c6fadbeaec)
* [jsk_pcl_ros_utils][polygon_magnifier] Support scale factor to  magnify polygon (`#2072 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2072>`_ )
  * [jsk_pcl_ros_utils][polygon_magnifier] support scale factor to magnify
* Fix website url for jsk_pcl_ros_utils (`#2071 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2071>`_ )
  - modified:   README.md
  - modified:   jsk_pcl_ros_utils/package.xml
* [jsk_pcl_ros_utils][polygon_magnifier] allow negative distance to magnify (`#2053 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2053>`_ )
  [jsk_pcl_ros_utils][polygon_magnifier] update docs
  [jsk_recognition_utils] add polygon_array_publisher.py / sample_polygon_array_publisher.launch
  [jsk_pcl_ros_utils] add sample / test for polygon_magnifier
* Generate README by script (`#2064 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2064>`_)
* [jsk_pcl_ros_utils][plane_rejector] add onInitPostProcess (`#2049 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2049>`_)
* [jsk_pcl_ros_utils][CMakeLists.txt] Suppress warning on build (`#2040 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2040>`_)
  * [jsk_pcl_ros_utils][CMakeLists.txt] remove comment out lines
  * [jsk_pcl_ros_utils][CMakeLists.txt] remove debug line
  * [jsk_pcl_ros_utils][CMakeLists.txt] comment out generate_messages
* [jsk_pcl_ros_utils] ensure super class functionality works (`#2043 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2043>`_ )
  * [jsk_pcl_ros_utils] ensure call onInitPostProcess() on DiagnosticNodelet
  * [jsk_pcl_ros_utils] ensure poke on callback in DiagnosticNodelet
* [jsk_pcl_ros_utils][centroid_publisher_nodelet] support polygon array (`#2038 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2038>`_ )
* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta

1.1.1 (2017-03-04)
------------------
* Remove unnecessary cmake messages (`#2010 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2010>`_)
* Contributors: Kentaro Wada

1.1.0 (2017-02-09)
------------------

1.0.4 (2017-02-09)
------------------
* scripts/evaluate_voxel_segmentation_by_gt_box.py: Cast to string to get correctly ns from rosparam (`#2016 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2016>`_ )
* Contributors: Kentaro Wada

1.0.3 (2017-02-08)
------------------
* Convert Voxel represented by MarkerArray to PointCloud (`#2012 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2012>`_ )
  * src/marker_array_voxel_to_pointcloud_nodelet.cpp
* Use bunny_marker_array.bag longer and high resolution (`#2011 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2011>`_ )
* Evaluate box/voxel segmentation with gt. box (`#1993 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1993>`_ )
  * Use longer rosbag for not-published /clock
  * Move evaluation scripts of box segmentation to jsk_pcl_ros_utils
* Re-enable tests in jsk_pcl_ros_utils (`#2008 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2008>`_ )
  * Fix index bag in point_indices_to_mask_image_nodelet
  * Use light rosbag for samples in jsk_pcl_ros_utils
  * Comment out pcl tests
* Contributors: Kentaro Wada

1.0.2 (2017-01-12)
------------------

1.0.1 (2016-12-13)
------------------
* package.xml : Fix dependency (jsk_data) of jsk_pcl_ros_utils
* Contributors: Kentaro Wada

1.0.0 (2016-12-12)
------------------
* Add PointCloudXYZToXYZRGB utility nodelet (`#1967 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1967>`_)
 * Test for PointCloudXYZToXYZRGB
 * Sample for PointCloudXYZToXYZRGB
* [jsk_pcl_ros_utils] Add subtract_point_indices (`#1952 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1952>`_)
* [jsk_pcl_ros_utils/add_point_indices] Add test  (`#1945 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1945>`_)
* [jsk_pcl_ros_utils] Removed dependencies of install_test_data.py (`#1949 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1949>`_)
* Contributors: Kentaro Wada, Iori Yanokura

0.3.29 (2016-10-30)
-------------------

0.3.28 (2016-10-29)
-------------------

0.3.27 (2016-10-29)
-------------------

0.3.26 (2016-10-27)
-------------------
* Stop using deprecated jsk_topic_tools/log_utils.h (`#1933 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1933>`_)
* [jsk_pcl_ros_utils/static_polygon_array_publisher] Fix typo (`#1916 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1916>`_)
* [jsk_pcl_ros_utils/plane_rejector_nodelet.cpp] Add allow_flip option to plane rejector (`#1876 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1886>`_)
* Contributors: Kentaro Wada, Iori Yanokura, Iori Kumagai

0.3.25 (2016-09-16)
-------------------

0.3.24 (2016-09-15)
-------------------

0.3.23 (2016-09-14)
-------------------

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
