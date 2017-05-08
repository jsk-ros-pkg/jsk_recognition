# PolygonMagnifier

![](./images/polygon_magnifier.png)

Magnify polygons by specified length.

## Subscribing Topic
* `~input` (`jsk_recognition_msgs/PolygonArray`)

  Input polygons

## Publishing Topic
* `~output` (`jsk_recognition_msgs/PolygonArray`)

  Output magnified polygons

## Parameters
* `~magnify_distance` (Double, default: `0.2`)

  Length to scale polygons. Default value `0.2` means the distance of each corresponding edges will be `0.2` m.
  If this value is less than `0`, output polygons are shrinked.

## Sample

``` bash
roslaunch jsk_pcl_ros_utils sample_polygon_magnifier.launch
```
