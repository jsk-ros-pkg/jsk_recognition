# ArithmeticImageOperator (AddImages, Subtract..)

Compute elementwise arithmetic operation with two images.

## Nodelets

* `jsk_perception/AddImages`
* `jsk_perception/SubtractImages`
* `jsk_perception/MultiplyImages`
* `jsk_perception/DivideImages`

## Subscribing Topic

* `~input/src1` (`sensor_msgs/Image`)
* `~input/src2` (`sensor_msgs/Image`)

  Input images that has arbitrary encodings.


## Publishing Topic

* `~output` (`sensor_msgs/Image`)

  Operation result.


## Parameters

* `~approximate_sync` (Bool, default: `false`)

  Approximately synchronize `~input/src1` and `~input/src2` if it's true.

* `~queue_size` (Int, default: `100`)

  Size of queue for synchronizing.

## Note

* Adding float images which contains NaN values.

  The operation will be such like `X + NaN = NaN`, `NaN + Y = NaN`.

* Adding mask images.

  Currently bitwise operations are not included.
  Please use `AddMaskImage` and `MultiplyMaskImage`.
