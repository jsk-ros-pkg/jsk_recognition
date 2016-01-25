# PolygonArrayColorHistogram

Compute color histogram of the region which is specified by 3-D Polygon.

## Subscribing Topics
* `~input` (`jsk_recognition_msgs/PolygonArray`)

  Input 3-D polygon array.
* `~input/image` (`sensor_msgs/Image`)

  Input image.
* `~input/info` (`sensor_msgs/CameraInfo`)

  Input camera info.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/HistogramWithRangeArray`)

  Histogram array. The order of the array is same to `~input` polygon array.

* `~debug/polygon_image` (`sensor_msgs/Image`)

  Debug image.

## Parameter
* `~bin_size` (default: `10`)

  The number of histogram bins.

* `~pixel_min_value` (default: `0`)
* `~pixel_max_value` (default: `180`)

  Minimum and maximum value of the histogram.

* `~debug_line_width` (default: `2`)

  Line width of debug image.
