# PointIndicesToMaskImage
## What Is This
![](images/point_indices_to_mask_image.png)

jsk\_pcl/PointIndicesToMaskImage generates mask image from `pcl_msgs/PointIndices`
of organized pointcloud and original `sensor_msgs/Image`.

## Subscribing Topic
* `~input` (`pcl_msgs/PointIndices`)

   Indices of the point cloud to mask.

* `~input/image` (`sensor_msgs/Image`)

   In order to know width and height of the original image, jsk\_pcl/PointIndicesToMaskImage requires
   input image.

## Publishing Topic

* `~output `(`sensor_msg/Image`)


   Mask image to get `~input` indices from the origina limage.
