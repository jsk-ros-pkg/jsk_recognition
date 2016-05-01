# AddPointIndices
add two different `pcl_msgs/PointIndices` into one indices.

## Subscribing Topic
* `~input/src1` (`pcl_msgs/PointIndices`)
* `~input/src2` (`pcl_msgs/PointIndices`)

  Input indices

## Publishing Topic
* `~output` (`pcl_msgs/PointIndices`)

  Output indices

## Parameters
* `approximate_sync` (Boolean, default: `false`)

  If this parameter is true, `~input/src1` and `~input/src2` are synchronized with
  approximate time policy.
