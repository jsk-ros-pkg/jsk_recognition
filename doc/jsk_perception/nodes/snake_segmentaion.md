# SnakeSegmentation
![](images/snake_segmentation.png)

Snake segmentation based on cvSnakeImage.

## Subscribing Topics
* `~input` (`sensor_msgs/Image`)

  Input image.

## Publishing Topics
* `~debug` (`sensor_msgs/Image`)

  Debug image.

## Parameters
* `~alpha` (double, default: `0.1`)

  weight of connectivity energy

* `~beta` (double, default: `0.1`)

  weight of curvature energy

* `~gamma` (double, default: `0.1`)

  weight of image energy

* `~window_size` (Integer, default: `3`)
* `~max_iterations` (Integer, default: `1000`)
* `~epsilon` (double, default: `0.1`)

  epsilon value of convergence