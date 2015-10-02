#!/bin/sh

OUTPUT=output.csv
rm $OUTPUT

variance=0.0001
outlier=0.1
for num in `seq 10 10 1000`
do
    rosrun jsk_pcl_ros bench_ransac_plane_estimation 100 $num 1000 $variance $outlier >> $OUTPUT
done
