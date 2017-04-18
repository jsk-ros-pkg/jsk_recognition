# EdgeDetector
Detect edge of input image.

## Subscribing Topic
* `image` (`sensor_msgs/Image`)

  Input image.It isn't need to be gray scale.

## Publishing Topic
* `~image` (`sensor_msgs/Image`)

  Edge image.

## Parameters
* `~threshold1` (Double, default: `100.0`)
* `~threshold2` (Double, default: `200.0`)
* `~apertureSize` (Integer, default: `3`)
* `~L2gradient` (Boolean, default: `false`)

  Parameters for [Canny()](http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#canny)

* `~apply_blur_pre` (Boolean, default: `true`)
* `~apply_blur_post` (Boolean, default: `false`)

  Parameters for [Blur()](http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#blur)

* `~postBlurSize` (Integer, default: `13`)
* `~postBlurSigma` (Double, default: `3.2`)

  Parameters for [GaussianBlur()](http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#gaussianblur)
