jsk_pcl_ros ROS Launch Files
============================

**Description:** jsk_pcl_ros

  
  
       jsk_pcl_ros
  
    

**License:** BSD

colorized_random_points_RF.launch
---------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros colorized_random_points_RF.launch

colorized_segmented_RF.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros colorized_segmented_RF.launch

euclidean_segmentation_sample.launch
------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros euclidean_segmentation_sample.launch

hinted_plane_detector_sample.launch
-----------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros hinted_plane_detector_sample.launch

hsi_color_filter_sample.launch
------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros hsi_color_filter_sample.launch

keypoints_publisher_sample.launch
---------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros keypoints_publisher_sample.launch

octree_change_detector_sample.launch
------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros octree_change_detector_sample.launch

pointcloud_screenpoint_sample.launch
------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pointcloud_screenpoint_sample.launch



This script displays which point the user pointed on a 2D screen in the 3D rviz.

Sample images below:

.. image:: launch/images/pointcloud_screenpoint_3people.png
  :width: 640

There are two parameters to input.

1. 'image' is used in showing a image view on the image_view2, and its camera_info parameter is used for changing the coordinates of points.
2. 'points' is the pointcloud source to estimate 3D points that the user wantedt to specify on a 2D screen.

For example,
By default, pointcloud_screenpoint takes camera image and point clouds from kinect.

.. code-block:: bash

  roslaunch jsk_pcl_ros pointcloud_screenpoint_sample.launch

You can mix wide_stereo camera and assembled_tilt_scan.

.. code-block:: bash

  roslaunch jsk_pcl_ros pointcloud_screenpoint_sample.launch image:=/wide_stereo/left points:=/tilt_laser_cloud2

Or, you can mix kinect camera and assembled_tilt_scan.

.. code-block:: bash

  roslaunch jsk_pcl_ros pointcloud_screenpoint_sample.launch image:=/camera/rgb points:=/tilt_laser_cloud2

pointclouds published by kinect

.. image:: launch/images/pointcloud_screenpoint_kinect.png
  :width: 640

pointclouds published by laser

.. image:: launch/images/pointcloud_screenpoint_laser.png
  :width: 640

amplifiered pointclouds published by laser

.. image:: launch/images/pointcloud_screenpoint_disparity_laser.png
  :width: 640


  

Contents
########

.. code-block:: xml

  <launch>
    <machine address="pr1012" name="c1" ros-package-path="$(env ROS_PACKAGE_PATH)" ros-root="$(env ROS_ROOT)">
      <env name="PATH" value="$(env PATH)" />
    </machine>
    <arg default="c1" name="cloud_machine" />
    <arg default="localhost" name="display_machine" />
    <arg default="/camera/rgb" name="image" />
    <arg default="/camera/rgb/points" name="points" />
    <include file="$(find jsk_pcl_ros)/launch/pointcloud_screenpoint.launch">
      <arg default="$(arg cloud_machine)" name="cloud_machine" />
      <arg default="$(arg display_machine)" name="display_machine" />
      <arg default="$(arg image)" name="image" />
      <arg default="$(arg points)" name="points" />
    </include>
    <node machine="localhost" name="display_point" output="screen" pkg="jsk_pcl_ros" type="pointcloud_screenpoint.l">
      <param name="~sensor_topic" value="$(arg image)/image_rect_color" />
      <param name="~ray_srv" value="/pointcloud_screenpoint_nodelet/screen_to_point" />
    </node>
    
    
    <node machine="$(arg display_machine)" name="image_view2" pkg="image_view2" type="image_view2">
      <remap from="image" to="$(arg image)/image_rect_color" />
      <remap from="camera_info" to="$(arg image)/camera_info" />
    </node>
    
    
  
    </launch>

rgb_color_filter_sample.launch
------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros rgb_color_filter_sample.launch

sample_610_clothes.launch
-------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros sample_610_clothes.launch

sample_drc_box.launch
---------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros sample_drc_box.launch

sample_subway.launch
--------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros sample_subway.launch

snapit_sample.launch
--------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros snapit_sample.launch

tf_transform_cloud.launch
-------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tf_transform_cloud.launch

tracking_sample.launch
----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tracking_sample.launch

attention_clipper_sample.launch
-------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros attention_clipper_sample.launch

background_subtraction.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros background_subtraction.launch

border_estimate.launch
----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros border_estimate.launch

calculate_normal.launch
-----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros calculate_normal.launch

depth_calibration.launch
------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros depth_calibration.launch

depth_error.launch
------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros depth_error.launch

euclidean_segmentation.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros euclidean_segmentation.launch

footstep_recognition.launch
---------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros footstep_recognition.launch

hinted_plane_detector.launch
----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros hinted_plane_detector.launch

hrp2jsknt_footstep_polygon.launch
---------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros hrp2jsknt_footstep_polygon.launch

hsi_color_filter.launch
-----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros hsi_color_filter.launch

keypoints_publisher.launch
--------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros keypoints_publisher.launch

laser_multi_plane_detection.launch
----------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros laser_multi_plane_detection.launch

laserscan_registration.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros laserscan_registration.launch

lazy_concatenate.launch
-----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros lazy_concatenate.launch

multisense_laser_listener.launch
--------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros multisense_laser_listener.launch

octree_change_detector.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros octree_change_detector.launch

openni2_local.launch
--------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros openni2_local.launch

openni2_remote.launch
---------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros openni2_remote.launch

openni_local.launch
-------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros openni_local.launch

openni_remote.launch
--------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros openni_remote.launch

organized_edge_detector.launch
------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros organized_edge_detector.launch

organized_multi_plane_segmentation.launch
-----------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros organized_multi_plane_segmentation.launch

pcl_roi_remote.launch
---------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pcl_roi_remote.launch

pcl_roi_robot.launch
--------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pcl_roi_robot.launch

pgstereo.launch
---------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pgstereo.launch

pointcloud_screenpoint.launch
-----------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pointcloud_screenpoint.launch



  Please see pointcloud_screenpoint_sample.launch for documentation.

  

Contents
########

.. code-block:: xml

  <launch>
    <machine address="localhost" name="localhost" />
    <arg default="localhost" name="cloud_machine" />
    <arg default="/openni/rgb" name="image" />
    <arg default="image_rect_color" name="image_type" />
    <arg default="$(arg image)/camera_info" name="camera_info" />
    <arg default="/openni/depth_registered/points" name="points" />
    <arg default="false" name="LAUNCH_OPENNI" />
    <arg default="true" name="USE_DEPTH_CREATER" />
    <arg default="false" name="USE_REGISTERED_DEPTH" />
    <arg default="false" name="USE_VIEW" />
    <arg default="false" name="USE_SYNC" />
    <arg default="false" name="PUBLISH_POINTS" />
  
    <arg name="inpoints" unless="$(arg USE_DEPTH_CREATER)" value="$(arg points)" />
    <arg if="$(arg USE_DEPTH_CREATER)" name="inpoints" value="/screenpoint_manager/points" />
  
    <include file="$(find openni_launch)/launch/openni.launch" if="$(arg LAUNCH_OPENNI)">
      <arg name="camera" value="openni" />
    </include>
  
    <node args="manager" machine="$(arg cloud_machine)" name="screenpoint_manager" output="screen" pkg="nodelet" respawn="true" type="nodelet" />
  
    <node args="load jsk_pcl/DepthImageCreator screenpoint_manager" clear_params="true" if="$(arg USE_DEPTH_CREATER)" machine="$(arg cloud_machine)" name="depth_image_creator_nodelet" output="screen" pkg="nodelet" respawn="true" type="nodelet">
      <remap from="~info" to="$(arg camera_info)" />
      <remap from="~input" to="$(arg points)" />
      <remap from="~output_cloud" to="/screenpoint_manager/points" />
      <rosparam>
        scale_depth: 5.0
        max_queue_size: 10
        use_fixed_transform: false
        use_service: false
        use_asynchronous: false
        use_approximate: true
      </rosparam>
    </node>
    <node args="load depth_image_proc/point_cloud_xyzrgb screenpoint_manager" clear_params="true" if="$(arg USE_REGISTERED_DEPTH)" machine="$(arg cloud_machine)" name="depth_image_proc_nodelet" output="screen" pkg="nodelet" respawn="true" type="nodelet">
      <remap from="rgb/camera_info" to="$(arg camera_info)" />
      <remap from="rgb/image_rect_color" to="$(arg image)/$(arg image_type)" />
      <remap from="depth_registered/image_rect" to="/depth_image_creator_nodelet/output" />
      <param name="qeueu_size" value="30" />
    </node>
  
    <group unless="$(arg USE_VIEW)">
    <node args="load jsk_pcl/PointcloudScreenpoint screenpoint_manager" clear_params="true" machine="$(arg cloud_machine)" name="pointcloud_screenpoint_nodelet" output="screen" pkg="nodelet" respawn="true" type="nodelet">
      <remap from="~points" to="$(arg inpoints)" />
      <rosparam>
        queue_size: 4
        crop_size: 10
        search_size: 16
        use_rect: false
        use_point_array: true
        use_point: true
        publish_point: true
      </rosparam>
      <param name="use_sync" value="$(arg USE_SYNC)" />
      <param name="publish_points" value="$(arg PUBLISH_POINTS)" />
    </node>
    </group>
  
    <group if="$(arg USE_VIEW)">
    <node name="screenpoint_view" ns="$(arg image)" output="screen" pkg="image_view2" respawn="true" type="image_view2">
      <remap from="image" to="$(arg image_type)" />
      <param name="autosize" value="true" />
    </node>
    <node args="load jsk_pcl/PointcloudScreenpoint screenpoint_manager" clear_params="true" machine="$(arg cloud_machine)" name="pointcloud_screenpoint_nodelet" output="screen" pkg="nodelet" respawn="true" type="nodelet">
      <remap from="~points" to="$(arg inpoints)" />
      <remap from="~point" to="$(arg image)/$(arg image_type)/screenpoint" />
      <remap from="~rect" to="$(arg image)/$(arg image_type)/screenrectangle" />
      <remap from="~point_array" to="$(arg image)/$(arg image_type)/screenpoint_array" />
      <rosparam>
        queue_size: 10
        crop_size: 10
        search_size: 16
        use_rect: false
        use_point_array: true
        use_point: true
        publish_point: true
      </rosparam>
      <param name="use_sync" value="$(arg USE_SYNC)" />
      <param name="publish_points" value="$(arg PUBLISH_POINTS)" />
    </node>
    </group>
  
    </launch>

pr2_pointcloud_error_visualization.launch
-----------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pr2_pointcloud_error_visualization.launch

pr2_tilt_laser_listener.launch
------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pr2_tilt_laser_listener.launch

pr2_tilt_laser_multi_plane_segmentation.launch
----------------------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros pr2_tilt_laser_multi_plane_segmentation.launch

region_growing_segmentation.launch
----------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros region_growing_segmentation.launch

rgb_color_filter.launch
-----------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros rgb_color_filter.launch

snapit.launch
-------------

.. code-block:: bash

  roslaunch jsk_pcl_ros snapit.launch

topic_tools.launch
------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros topic_tools.launch

tracking.launch
---------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tracking.launch

visualize_plane_distance.launch
-------------------------------

.. code-block:: bash

  roslaunch jsk_pcl_ros visualize_plane_distance.launch

demo_tower.launch
-----------------

.. code-block:: bash

  roslaunch jsk_pcl_ros demo_tower.launch

kinect.launch
-------------

.. code-block:: bash

  roslaunch jsk_pcl_ros kinect.launch

tower_pcl.launch
----------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tower_pcl.launch

tower_tf.launch
---------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tower_tf.launch

tower_web.launch
----------------

.. code-block:: bash

  roslaunch jsk_pcl_ros tower_web.launch

