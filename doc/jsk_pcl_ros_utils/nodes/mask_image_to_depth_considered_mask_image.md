# MaskImageToDepthConsideredMaskImage
## What Is This
![](images/mask_image_to_depth_considered_mask_image.png)

jsk_pcl/MaskImageToDepthConsideredMaskImage extracts directed area of mask image in the order of depth from `sensor_msgs/PointCloud2` and `sensor_msgs/Image` of mask image. Example is at jsk_pcl_ros/launch/extract_only_directed_region_of_close_mask_image.launch .

## Subscribing Topic

* `~input` (`sensor_msgs/PointCloud2`)

  Depth information of image. Width and height of this data must be same with ~input/image.

* `~input/image` (`sensor_msgs/Image`)

  Input mask image.

* `~input/maskregion` (`sensor_msgs/Image`)

  Input mask region.(To use interactively, use interaction_mode:grabcut_rect of image_view2.)

## Publishing Topic

* `~output` (`sensor_msg/Image`)

  Output mask Image.  Points at close range is extracted.

## Parameter

* `~extract_num` (Int, default: `400`)

  Num of extract points in mask image.

* `~use_mask_region` (Bool, default: `True`)

  Whether use mask region option or not. If true, only selected region of mask image is extracted.

* `~in_the_order_of_depth` (Bool, default: `True`)

  Extracted points are in the order of the depth image if enabled.

* `~approximate_sync` (Bool, default: `False`)

  Use approximate synchronization policy instead of exact synchronization if enabled.

* `~queue_size` (Int, default: `100`)

  Queue size of input topics
