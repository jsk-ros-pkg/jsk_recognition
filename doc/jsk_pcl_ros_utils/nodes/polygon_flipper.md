# PolygonFlipper

Flip `jsk_recognition_msgs/PolygonArray` to specified sensor_frame.

## Subscribing Topic
* `~input/polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~input/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

  Input polygons.
## Publishing Topic
* `~output/polygons` (`jsk_recognition_msgs/PolygonArray`)
* `~output/coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

  Output flipped polygons which look at the origin of sensor_frame.
## Parameter
* `~sensor_frame` (String)

   frame_id of sensor for polygons to look at.
