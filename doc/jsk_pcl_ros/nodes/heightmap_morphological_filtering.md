# HeightmapMorphologicalFiltering
![](images/heightmap_morphological_filtering.png)

Apply morphological fintering and average filter to fill small holes in pointcloud
which is represented as heightmap.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input heightmap. Hole should be represented as `-FLT_MAX` or `nan`.
* `~output/config` (`jsk_recognition_msgs/HeightmapConfig`)

  Config topic.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Output heightmap.

## Parameters
* `~max_queue_size` (Integer, default: `10`):

  Max queue size of subscription callback.
* `~mask_size` (Integer, default: `2`):

  Size of kernel operator of average filtering.
* `~max_variance` (Double, default: `0.1`):

  Allowable max variance in kernel operator
* `~smooth_method` (default: `average`)

  You can choose method of smoothing from `average` and `bilateral`.
* `~bilateral_filter_size` (Integer, default: `7`)

  Kernel size of bilateral filtering.
* `~bilateral_sigma_color` (Double, default: `35`)

  filter sigma of color space.
* `~bilateral_sigma_space` (Double, default: `5`)

  filter sigma of coordinate space.
