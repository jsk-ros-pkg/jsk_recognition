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
>> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up -j3 \
  vision_opencv/cv_bridge \
  vision_opencv/image_geometry \
  image_common/image_transport
