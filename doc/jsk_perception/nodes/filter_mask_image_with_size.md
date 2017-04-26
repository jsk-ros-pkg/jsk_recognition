# FilterMaskImageWithSize

![](images/filter_mask_image_with_size.gif)

Filter mask image with the size of white region relative to image size
and reference mask size.


## Subscribing Topic

* `~input` (`sensor_msgs/Image`)

  Input mask.

* `~input/reference` (`sensor_msgs/Image`)

  Reference mask. Only subscribed if `~use_reference` is `true`.


## Publishing Topic

* `~output` (`sensor_msgs/Image`)

  Filtered mask.


## Parameters

* `~use_reference` (Bool, default: `false`)

  If `true`, `~input/reference` is subscribed,
  and rosparams: `~min_relative_size` and `~max_relative_size` are enabled.

* `~min_size`, `~max_size` (Float, default: `0`, `1`)

  Size threshold of white region of input mask relative to image size.

* `~min_relative_size`, `~max_relative_size` (Float, default: `0`, `1`)

  Size threshold relative to the reference mask's white region.
  Enabled with `~use_reference:=true`.

* `~approximate_sync` (Bool, default: `false`)

  Approximately synchronize topics if it is `true`.

* `~queue_size` (Int, default: `100`)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.


## Sample

```bash
roslaunch jsk_perception sample_filter_mask_image_with_size.launch
```
