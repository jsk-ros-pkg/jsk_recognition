# jsk\_pcl\_ros


## Introduction
jsk\_pcl\_ros is a package to provide some programs using [pcl](http://pointclouds.org).

This package provides some programs as nodelet.

## nodelets
### jsk\_pcl/ParticleFilterTracking
####What Is This

This nodelet will track the target pointcloud which you set in rviz.

####Sample

run the below command.

```
roslaunch jsk_pcl_ros tracking_groovy.launch # (When use groovy)  
roslaunch jsk_pcl_ros tracking_hydro.launch  #(When use hydro)
```

and run the rviz.


```
rosrun rviz rviz
```

After rviz brang up, select "Add New Panel" and choose jsk_rviz_plugins' SelectPointCloudPublishAction Panel. Now, the new Panel will appear at left display.

Push the "Select" button at the top bar , drag and surround the target poincloud which you want to track in the rectangle area.Then, finally, push the "SelectPointCloudPublishActoin" button at SelectPointCloudPublishAction Panel. The tracker will start tracking the target.


### jsk\_pcl/ResizePointsPublisher
### jsk\_pcl/PointcloudScreenpoint
### jsk\_pcl/DepthImageCreator
### jsk\_pcl/EuclideanClustering
### jsk\_pcl/ClusterPointIndicesDecomposer
### jsk\_pcl/ClusterPointIndicesDecomposerZAxis
### jsk\_pcl/CentroidPublisher
### jsk\_pcl/VoxelGridDownsampleManager
### jsk\_pcl/VoxelGridDownsampleDecoder
### jsk\_pcl/Snapit
### jsk\_pcl/KeypointsPublisher
### jsk\_pcl/HintedPlaneDetector
### jsk\_pcl/ObjectChangeDetector
####What Is This

This nodelet will publish the difference of sequential pointcloud. You can get the newly generated pointclouds.

Difference with pcl_ros/SegmentDifference refer https://github.com/jsk-ros-pkg/jsk_recognition/pull/67

####Sample

####Speed

### jsk\_pcl/TfTransformCloud
####What Is This

This nodelet will republish the pointcloud which is transformed with the designated frame_id.

####Sample
