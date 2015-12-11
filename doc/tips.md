# Tips

## Cannot compile jsk_recognition because of "memory allocation error"
`jsk_pcl_ros` and `jsk_perception` requires much memory to be compiled because of PCL.
On average, each cpp file requires 2.5GB memory to compile.

If your machine does not have enough memory, please use smaller number of CPUs to compile

```
catkin build -p1 -j1
```

## Want to use with newer or older PCL or OpenCV
You just need to re-compile all the packages which depend on PCL or OpenCV.
For example, if yor are using pcl 1.8, you need to recompile
`jsk_pcl_ros`, `pcl_ros`, `pcl_conversions`, `navfn` and so on.

`rosdep what-needs` may help you.

```
$ rosdep what-needs libpcl-all
octomap_server
pcl_conversions
snap_map_icp
pcl_ros
```
