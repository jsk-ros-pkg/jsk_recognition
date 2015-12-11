# NormalEstimationOMP
This nodelet is almost same to `pcl/NormalEstimationOMP` of `pcl_ros` package,
but it can handle timestamp correctly.

## Subscribing Topic
* `~input` (`sensor_msgs/PointCloud2`)

  Input pointcloud.

## Publishing Topic
* `~output` (`sensor_msgs/PointCloud2`)

  Output pointcloud, point type is `pcl::Normal`.
* `~output_with_xyz` (`sensor_msgs/PointCloud2`)

  Output pointcloud, point type is `pcl::XYZRGBNormal`.

* `~output/latest_time` (`std_msgs/Float32`)

  latest computation time

* `~output/average_time` (`std_msgs/Float32`)

  average computation time

## Parameters
* `~k_search`

  K search parameter for normal estimation
* `~radius_search`

  Radius search parameter for normal estimation
