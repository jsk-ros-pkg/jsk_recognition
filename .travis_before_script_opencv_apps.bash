#!/usr/bin/env bash

######################################################
# Replace opencv_apps's CMakeLists.txt for hydro
######################################################
cd ~/ros/ws_$REPOSITORY_NAME/src/ros-perception/opencv_apps

# because of unreleased sensor_msgs::image::encodings on hydro.
sed -i '/opencv_apps_add_nodelet(lk_flow /s/^/# /g' CMakeLists.txt

# because sensor_msgs does not have 'CompressedImageConstPtr' on hydro.
sed -i '/opencv_apps_add_nodelet(simple_compressed_example /s/^/# /g' CMakeLists.txt
