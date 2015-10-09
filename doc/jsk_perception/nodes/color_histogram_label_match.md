# ColorHistogramLabelMatch
![](images/color_histogram_label_match.png)

Compute similar region of image to specified histogram based on superpixels image.

Sample is `color_histogram_label_match_sample.launch`.

## Input Topic
* `~input/histogram` (`jsk_recognition_msgs/ColorHistogram`)

  Reference histogram.
* `~input` (`sensor_msgs/Image`)

  Input image. This image should be bgr8 or rgb8 image.
* `~input/label` (`sensor_msgs/Image`)

  Label of ~input image. Label image should be int32 image.
* `~input/mask` (`sensor_msgs/Image`)

  Mask image of ~input image. Only masked region is taken into account.

## Publishing Topic
* `~output/extracted_region` (`sensor_msgs/Image`)

  Result of correlation computation as mask image.
* `~output/coefficient_image` (`sensor_msgs/Image`)

  Result of correlation computation as float image.
* `~debug` (`sensor_msgs/Image`)

  Debug image
## Parameters
* `~coefficient_method`

  Method to compute coefficient
* `~max_value` (Default: `255`)
* `~min_value` (Default: `0`)

  Maximum and minimum index of histogram
* `~masked_coefficient` (Default: `0.0`)

  Value to fill masked region
* `~threshold_method`

  Method to binalize coefficient image.
* `~coef_threshold`

  Threshold used in binalization.
* `~use_mask` (Default: `false`)

  Do not use mask image if this parameter is false.
