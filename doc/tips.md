# Tips

## Cannot compile jsk_recognition because of "memory allocation error"
`jsk_pcl_ros` and `jsk_perception` requires much memory to be compiled because of PCL.
On average, each cpp file requires 2.5GB memory to compile.

If your machine does not have enough memory, please use smaller number of CPUs to compile

```
catkin build -p1 -j1
```
