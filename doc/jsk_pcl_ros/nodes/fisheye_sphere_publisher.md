# FisheyeSpherePublisher
![](images/fisheye_sphere1.png)

Show the sphere point cloud generated from fisheye image.
This was tested with Prosilica GC 2450C + nm30 lens

## Subscribing Topics
* `~input` (`sensor_msgs/Image`)

  Fisheye Image

## Publishing Topics
* `~output` (`sensor_msgs/PointCloud2`)

  Sphere pointcloud.


## Sample

```bash
roslaunch jsk_pcl_ros sample_fisheye_sphere_publisher.launch
```
