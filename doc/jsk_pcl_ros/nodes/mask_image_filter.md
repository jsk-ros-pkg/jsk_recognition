# MaskImageFilter
![](images/mask_image_filter.png)

Extract indices of pointcloud which is masked by mask image. The pointcloud is no need to be organized.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input point cloud.
* `~input/mask` (`sensor_msgs/Image`)

  Mask image.
* `~input/camera_info` (`sensor_msgs/CameraInfo`)

  Camera parameters of the image.

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Indices of the points masked by `~input/mask`.

