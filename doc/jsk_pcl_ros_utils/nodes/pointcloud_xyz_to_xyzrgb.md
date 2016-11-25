# PointCloudXYZToXYZRGB

## What is this?

Node to convert fields of `sensor_msgs/PointCloud2` from `XYZ` to `XYZRGB`.

## Subscribing Topic

* `~input` (`sensor_msgs/PointCloud2`)

  Input cloud whose field is `XYZ`.

## Publishing Topic

* `~output` (`sensor_msgs/PointCloud2`)

  Output cloud whose field is `XYZRGB`.
