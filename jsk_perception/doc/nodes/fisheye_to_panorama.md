# FisheyeToPanorama
This nodelet will publish Rectified or Panoramized Fisheye Image.
We recomend you to set scale factor as small as possible to reduce calculation.
This  was tested with Prosilica GC2450C and NM30 lens.
Below pictures show rectify image system.
![](images/fisheye_readme.png)

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input mask images.
## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Rectified or Panoramized Image

* `~output_biliner` (`sensor_msgs/Image`)

  When Simaple Panorama Mode, publish Panoramized Image

## Parameters
* `~use_panorama` (Bool, default: `false`)

  If true=> publish Parnorama View Image
  If false=> publish Rectified View Image

* `~simple_panorama` (Bool, default: `false`)

  This is effective only when use_panorama = true
  If true => show Simple Panorama View
  If false => show Calcurated Panorama View
