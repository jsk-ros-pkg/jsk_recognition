#!/usr/bin/env bash

######################################################
# Install PCL 1.8
######################################################

if [ ! -e /usr/local/include/pcl-1.8/pcl/pcl_base.h ]; then
  cd /tmp

  version="1.8.0rc2"
  url="https://github.com/PointCloudLibrary/pcl/archive/pcl-${version}.tar.gz"
  fname=pcl-${version}.tar.gz

  wget $url -O $fname
  tar zxf $fname

  cd pcl-pcl-${version}
  mkdir build
  cd build
 
  # pcl::CropBox does not work properly in kinetic
  # with PCL_ENABLE_SSE:BOOL=TRUE flag
  # https://github.com/PointCloudLibrary/pcl/pull/1917 
  if [ $(lsb_release -c -s) = "xenial" ]; then
    cmake -DCMAKE_BUILD_TYPE=Release -DPCL_ENABLE_SSE:BOOL=FALSE ..
  else
    cmake -DCMAKE_BUILD_TYPE=Release ..
  fi
  make -j2
  sudo make -j2 install
fi


######################################################
# Setup dependencies to rebuild from source
######################################################
sudo -H pip install -q rosinstall_generator

rosinstall_generator --tar --rosdistro $ROS_DISTRO \
  pcl_conversions \
  pcl_ros \
  octomap_server \
>> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up -j4
