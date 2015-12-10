# PolygonArrayColorLikelihood

Compute polygon likelihood based on distance of histograms.

## Subscribing Topics
* `~input/polygons` (`jsk_recognition_msgs/PolygonArray`)

  Input polygons.
* `~input/histograms` (`jsk_recognition_msgs/HistogramWithRangeArray`)

  Color histogram of input polygons.
* `~input/reference` (`jsk_recognition_msgs/HistogramWithRange`)

  Reference color histogram.

## Publishing Topics
* `~output` (`jsk_recognition_msgs/PolygonArray`)

  Output polygons with updated likelihood field.

## Parameters
* `~approximate_sync` (default: `false`)

  Approximately synchronize `~input/polygons` and `~input/histograms`.

* `~max_queue_size` (default: `10`)

  Max queue size
