#!/usr/bin/env bash

######################################################
# Install OpenCV 3
######################################################
sudo -H apt-get install -y -q -qq ros-$ROS_DISTRO-opencv3


######################################################
# Setup dependencies to rebuild from source
######################################################
sudo -H pip install -q rosinstall_generator

rosinstall_generator --tar --rosdistro $ROS_DISTRO \
  cv_bridge \
  image_geometry \
  image_transport \
  image_view2 \
  jsk_data \
  jsk_topic_tools \
  opencv_apps \
>> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up -j4
