# MorphologicalOperator
![](images/erode_mask_image.png)
![](images/dilate_mask_image.png)
![](images/morphological_operator.png)

Apply morphological transformations.

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Output transformed image.

## Parameters
* `~method` (`0`, `1` or `2`, default: `0`)

  Method to transform image. 0 means rectangular box model,
  1 meand cross model and 2 means ellipse.

* `~size` (Integer, default: `1`)

  Size to transform.

* `~iterations` (Integer, default: `1`)

  Iterations of transforming image.

