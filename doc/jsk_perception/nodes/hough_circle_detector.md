# HoughCircleDetector
Detect circles in input image.

## Subscribing Topic
* `image` (`sensor_msgs/Image`)

  Input image.

## Publishing Topic
* `~output` (`jsk_recognition_msgs/Circle2DArray`)

  Detected circles.

* `~image_marker` (`image_view2/ImageMarker2`)

  Detected circles visualized on [image_view2](https://github.com/jsk-ros-pkg/jsk_common/tree/master/jsk_ros_patch/image_view2).

## Parameters
* `~dp` (Integer, default: `2`)
* `~edge_threshold` (Double, default: `200.0`)
* `~voting_threshold` (Double, default: `100.0`)
* `~min_circle_radius` (Integer, default: `0`)
* `~max_circle_radius` (Integer, default: `0`)

  Parameters for [HoughCircles()](http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#houghcircles)

* `~gaussian_blur_size` (Integer, default: `9`)

  Parameters for [GaussianBlur()](http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#gaussianblur).This parameter should be odd number.

* `~gaussion_sigma_x` (Double, default: `2.0`)
* `~gaussion_sigma_y` (Double, default: `2.0`)

  Parameters for [GaussianBlur()](http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#gaussianblur).
