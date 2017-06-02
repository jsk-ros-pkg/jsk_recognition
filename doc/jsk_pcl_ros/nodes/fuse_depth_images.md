# FuseDepthImages

![](images/fuse_depth_images.jpg)

Do sensor fusions by multiple depth images.
For transformation of depth from one to another, you can use `jsk_pcl/DepthImageCreator`.
See `sample_fuse_depth_image.launch` for detail.

## Subscribing Topic

See rosparam `~input_topics`.


## Publishing Topic

* `~output` (`sensor_msgs/Image`)

  Output fused depth image.


## Parameters

* `~input_topics` (String array, required)

  Input depth image topics.

## Sample

```
roslaunch jsk_pcl_ros sample_fuse_depth_image.launch
```
