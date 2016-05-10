# TfTransformCloud

## Description

Transforms a point cloud in a given target TF frame.  
Refer [discussion at Github issue](https://github.com/jsk-ros-pkg/jsk_recognition/pull/70)

## Subscribing Topic

- `~input` (`sensor_msgs/PointCloud2`)

Input point cloud

## Publishing Topic

- `~output` (`sensor_msgs/PointCloud2`)

Transformed point cloud which has `~target_frame_id` as `header.frame_id`

## Parameters

- `~target_frame_id` (String, default: `base_footprint`)

Parent TF frame to be transformed

## Sample

```xml
<launch>
  <include file="$(find openni_launch)/openni_launch.launch" />
  <node pkg="tf" type="static_transform_publisher"
        args="10 20 30 20 10 20 /camera_link /tf_test_link 100" />
  <node name="pc_transformer" pkg="nodelet" type="nodelet"
        args="load jsk_pcl/TfTransformCloud /camera_nodelet_manager"
        output="screen" />
    <remap from="~input" to="/camera/depth/points" />
    <rosparam>
      target_frame_id: /tf_test_link
    </rosparam>
  </node>
</launch>
```
