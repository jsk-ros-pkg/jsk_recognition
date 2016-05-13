# ColorizeDistanceFromPlane

![](images/colorize_distance_from_plane.png)

## What Is This

Colorize points based on distance from planes.
This is usefull for calibration.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input point cloud.
* `~input_coefficients` (`jsk_recognition_msgs/ModelCoefficientsArray`)

  Input model coefficients. This topic is used only for synchronizing.
* `~input_polygons` (`jsk_recognition_msgs/PolygonArray`)

  Input plane.

## Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Output point cloud.
