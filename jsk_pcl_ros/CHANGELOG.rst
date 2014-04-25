^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_pcl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2014-04-25)
------------------
* fixed compile error jsk_pcl_ros
* Contributors: Kei Okada, Ryohei Ueda, Yuto Inagaki

0.1.3 (2014-04-12)
------------------
* add depends to visualization_msgs
* delete lines for refactoring the tracking
* add RGB color
* fill point_cloud field
* Contributors: Ryohei Ueda, Kei Okada, Yuto Inagaki

0.1.2 (2014-04-11)
------------------
* use find_moduel to check catkin/rosbuild to pass git-buildpackage
* Contributors: Kei Okada
* add CallPolygon.srv for `jsk-ros-pkg/jsk_smart_apps#17 <https://github.com/jsk-ros-pkg/jsk_smart_apps/issues/17>`_
* Contributors: Yuto Inagaki

0.1.1 (2014-04-10)
------------------
* add depend_tag for pcl_conversions and not needed tags
  delete not needed tags
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: catch runtime error in order to ignore error from tf and so on
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: use SlicedPointCloud in VoxelGridDownsampleDecoder and use NODELET_** macros
  instead of ROS_** macros
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_:  use SlicedPointCloud in VoxelGridDownsampleManager
* `#31 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/31>`_: add new message for VoxelGrid{Manager,Decoder}: SlicedPointCloud.msg
* replacing image_rotate namespace with jsk_pcl_ros because of porting
* fix package name of dynamic reconfigure setting file
* use ROS_VERSION_MINIMUM
* use TF2_ROS_VERSION instead of ROS_MINIMUM_VERSION macro
* use tf2::BufferClient on groovy
* add cfg file for image_rotate dynamic reconfigure
* porting image_rotate_nodelet from image_pipeline garamon's fork.
  this version of image_rotate supports tf2 and nodelet.
* add rosdepend to prevent pointcloud_screenpoint_nodelet error
* use jsk nodelet mux for pcl roi
* add arg to set nodelet manager name
* use the same nodelet manager as openni
* `#20 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/20>`_: implement PointCloudThrottle and ImageMUX, ImageDEMUX and ImageThrotle
* add sensor_msgs dependency to message generation
* Merge remote-tracking branch 'refs/remotes/garaemon/add-message-dependency-to-jsk-pcl-ros' into garaemon-avoid-roseus-catkin-bug
  Conflicts:
  jsk_pcl_ros/catkin.cmake
* change the location of generate_messages and catkin_package of jsk_pcl_ros
* add sensor_msgs depdendency to jsk_pcl_ros's message generation
* `#8 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/8>`_: remove delay pointcloud nodelet
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: remove unused comment
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: remove unused cpp source codes, now they are automatically generated from single_nodelet_exec.cpp.in
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: automatically generate the single nodelet programs on rosbuild
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: rename resize_points_publisher to resize_points_publisher_nodelet according to naming convention
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: fix endmacro syntax
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: automatically generate cpp codes in catkin build
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: add quotes to the template file
* `#15 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/15>`_: add a template file to run single nodelet
* add pcl_conversions to jsk_pcl_ros
* add eigen_conversions to jsk_pcl_ros dependency
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: specify package.xml by fullpath
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: add pcl to dependency if distro is groovy
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: pcl is not a catkin package
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: fix if sentence order
* `#11 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/11>`_: depend pcl catkin package in groovy
* listed up nodelets provided by jsk_pcl_ros
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: removed icp_server, it's just a sample program
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: remove LazyConcatenater and PointcloudFlowrate from CMakeLists.txt
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: remove LazyConcatenater and PointcloudFlowrate from jsk_pcl_nodelets.xml
* `#4 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/4>`_: removed LazyConcatenater and PointcloudFlowrate
* fix depend package -> rosdep name
* add keypoints publisher; first supported only nerf
* add code for using GICP if using hydro
* add PolygonArray.msg for catkin build system
* adding header
* adding more nodelet modules for catkin
* adding CallSnapIt.srv
* add tf topic name parameter
* add pcl roi launch files
* add base_frame parameter in voxel_grid_downsample
* adding special message for polygon array
* adding hinted plane detector to xml nodelet list
* enable use_point_array of screenpoint
* add include <pcl_conversions/pcl_conversions.h> for groovy
* use pcl_conversions for hydro, see http://wiki.ros.org/hydro/Migration#PCL
* fix wrong commit on
* forget to commit, sorry
* add SnapItRequest to add_message_files
* adding sample for hinted plane detector
* adding HintedPlaneDetector and pointcloudScreenpoint supports converting array of 2d points into 3d
* adding HintedPlaneDetector and pointcloudScreenpoint supports converting array of 2d points into 3d
* publishing marker as recognition result
* implemented snapit for cylinder model
* adding height field
* adding cylinder parameters
* supporting cylinder model fitting
* fix for groovy with catkin
* setting axis when snap to the plane
* fixing transformation concatenation
* adding new module: SnapIt
* fix issue `#268 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/268>`_, run sed only when needed
* does not publish if the grid is empty
* change the default value
* change the default value
* adding initial ROI
* adding initial ROI
* not cahing old points
* supporting the change of the voxel num
* supporting the change of the voxel num
* supporting the change of the voxel num
* supporting the change of the voxel num
* not remove previous pointcloud as long as possible
* change the default value to 300
* supporting frame_id
* using tf
* adding decoder for voxel grid downsample manager
* adding message
* adding voxel grid downsample manager
* adding voxel_grid_downsample_manager
* supporting dynamic reconfigure
* adding lazy concatenater
* adding lazy concatenate sample
* adding lazy_concatenater
* debug RGBColorFilter and HSVColorFIlter for hydro
* adding pointcloud_flowrate nodelet skelton
* adding pointcloud_flowrate nodelet skelton
* compile pointcloud_flowrate executable
* executable to run pointcloud_flowrate
* tracking.launch change to tracking_hydro.launch and tracking_groovy.launch
* add load_manifest for rosbuild
* fix filtering range when min value is grater than max value
* fix filter name
* add rgb filter
* add mutex::scoped_lock in particle_filter_tracking
* debug in renew_tracking.py ROS_INFO -> rospy.loginfo
* add scripts/renew_trakcing.py launch/tracking.launch
* use SetPointCloud2
* add particle filter trackig node/nodelt with SetPointCloud2.srv
* fix pointcloud_scrennpoint.cpp to use jsk_pcl_ros -> jsk_pcl, by k-okada
* enable respawning
* add depends to pcl_msgs
* adding icp server
* adding TOWER_LOWEST2
* support both catkin/rosbuild
* update catkin makefile, add _gencpp, _gencfg
* support both catkin/rosbuild
* add_dependences to jsk_pcl_ros_gencpp
* pcl -> pcl_msgs for pcl-1.7 (hydro), but use sed to force change pcl/ namespace for groovy
* hydro migration, pcl 1.7 is independent from ros, see http://wiki.ros.org/hydro/Migration
* use USE_ROSBUILD for catkin/rosbuild environment
* starting with the middle tower
* fixing typo
* fixing typo
* using positoin from /origin, instead of from robot frame id
* added code for running centroid_publishers to publish segmented point cloud centroids
* update the position parameter for the demo
* fixing the rotatio of camera
* update the index of tower, plate, using enum in srv
* adding service to move robot with just index
* update the parameter and the axis
* fix to move robot to the goal tower
* update to run with eus ik server
* resolve position of each tf
* set the quality of the mjpeg server 100
* fixing message of the modal of alert
* block the tower already having plates
* adding debug message
* adding empty function to move robot
* adding graph
* adding service type to move robot
* smaller fonts
* adding cluster num on debug layer
* adding the number of the clusters
* update
* update the message
* adding more states for hanoi-tower
* small fixes
* adding service to pickup tower
* adding text shadow
* click detection by service call
* cenrerize button
* adding help modal
* track the window size
* adding html to redirect to tower_detecct_viewer
* centerize the image
* centerize the image
* adding state
* introducing state machine
* detecting clicked cluster
* using tower_detect_viewer_server
* providing a class
* adding some web related files
* using filled flag
* update params for lab room
* specifying tf_timeout of image_view2
* not subscribing topic to refer timestamp
* fixing header timestamp
* using some topic to refer timestamp
* supporting marker id
* update
* update topic to use image_view2's image
* fixing draw_3d_circle
* add script to draw circle on image_view2
* using location.hostname for the IP address
* adding www directory for tower_detect brawser viewer
* adding a launch file to launch mjpeg_server
* adding CentroidPublisher
* empty CentroidPublisher class
* implementing z axis sorting
* more effective implementation
* more information about resetting tracking
* fixing registration parameter
* adding nodelet skelton cpp
* adding cluster_point_indices_decomposer_z_axis.cpp
* adding sortIndicesOrder as preparation to customize ordering technique
* adding new nodelet ClusterPointIndicesDecomposer
* adding more methods
* adding skelton class to decompose ClusterPointIndices
* adding license declaration
* adding launch file to examin euclidean segmentation
* fixing label tracking
* refactoring
* refactoring
* refactoring
* supporting label_tracking_tolerance
* refactoring
* implementing labeling tracking
* calculate distance matrix
* adding one more color
* refactoring
* fixing compilation warning
* calculate centroids at the first frame
* fixing indentation
* using static colors to colorize clustered pointclouds
* removing noisy output
* removing invalid comments
* supporting dynamic reconfigure for euclidean clustering
* fixing rotation
* adding /origin and /table_center
* adding two lanch files
* adding top level launch
* openni.launch with depth_registered=true
* fix missing dependancy
* update hsv_color_filter.launch
* add USE_REGISTERER_DEPTH argument to pointcloud_screenpoint.launch
* remove env-loader (localhost do not need env-loader)
* update parameter use_point false -> true
* add same parameters to not USE_VIEW
* fix strequal ROS_DISTRO env
* use ROS_Distributions instead of ROS_DISTRO for electric
* fix for electric
* add USE_SYNC parameter to pointcloud_screenpoint.launch
* update pointcloud_screnpoint.launch
* merged image_view2/points_rectangle_extractor.cpp to pointcloud_screenpoint
* add EuclideanClustering [`#119 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/119>`_]
* copy pcl_ros/filters/filter to jsk_pcl_ros directory due to https://github.com/ros-perception/perception_pcl/issues/9, [`#119 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/119>`_]
* add catkin.cmake, package.xml for groovy, remove nodelt depends on pcl_ros::Filter https://github.com/ros-perception/perception_pcl/issues/9
* fix description comment
* remove cv_bridge
* add sample code for using
* add lisp code for using pointcloud in roseus
* use tf::Quaternion instead of btQuaternion
* comment out pcl_ros/features/features.cpp
* libtbb -> tbb , see rosdep/base.yaml
* change rodep name:libtbb to tbb
* update index.rst,conf.py by Jenkins
* fix: high load of screenpoint
* fix: change dynamic config
* fix: variable range of hue
* delete obsolated files
* rewrite color_filter and color_filter_nodelet for PCL 1.5 and later
* update sample for color_filter
* update index.rst,conf.py by Jenkins
* changed arg setting for launch from pr2.launch using mux
* update index.rst,conf.py by Jenkins
* fix: for using pcl_ros/feature class
* changed kinect's name from camera to openni
* fix: depth_image_creator added to nodelet
* use machine tag with env-loader
* comment out old pcl modules
* remove machine tag, which is not used
* fix for compiling fuerte and electric
* fix row_step and is_dense variables for resized point cloud
* added service of switching topic for depth_image_creator
* update index.rst,conf.py by Jenkins
* outout launchdoc-generator to build directry to avoid svn confrict
* remove jskpointcloud dependency from jsk_pcl_ros
* copy depth_image_creator from unreleased
* add jsk_pcl_ros (copy from unreleased repository)
* Contributors: JSK applications, Kei Okada, Ryohei Ueda, aginika, chen, furushchev, furuta, inagaki, k-okada, kazuto, roseus, ueda, wesley, youhei
