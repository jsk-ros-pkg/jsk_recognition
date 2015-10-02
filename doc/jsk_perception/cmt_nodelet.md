# jsk\_perception/CMTNodelet
Rectangle Tracker Using libcmt.

![](https://cloud.githubusercontent.com/assets/3803922/8546574/e7044610-24f3-11e5-8e2e-303eabc599c4.gif)

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image and it's used to know original width and height.

* `~set_rect` (`jsk_recognition_msgs/Rect`)

  If you publish some rect, tracker will reset and restart tracking.

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Output tracking result image.

#### Parameters
* `~show_window` (Bool, default: `false`)
* `~topleft_x` (Int, default: `100`)
* `~topleft_y` (Int, default: `100`)
* `~bottomright_x` (Int, default: `200`)
* `~bottomright_y` (Int, default: `200`)
