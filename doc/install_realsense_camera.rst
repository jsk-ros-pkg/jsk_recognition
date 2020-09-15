Install RealSense camera
========================

- SDK
-- Intel Realsense SDK 2.0 (librealsense): https://github.com/IntelRealSense/librealsense
-- ROS Wrapper for Intel® RealSense™ Devices (realsense-ros): https://github.com/IntelRealSense/realsense-ros


- Data Sheet of RealSense Products
-- T265: https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf
-- D400 Series: https://www.intelrealsense.com/wp-content/uploads/2020/06/Intel-RealSense-D400-Series-Datasheet-June-2020.pdf
-- SR300: https://software.intel.com/sites/default/files/managed/0c/ec/realsense-sr300-product-datasheet-rev-1-0.pdf

- Legacy Devices
-- If you use Legacy Devices (F200, R200, LR200 and ZR300), please use `old librealsense <https://github.com/IntelRealSense/librealsense/tree/v1.12.1>`_._
-- Data Sheet R200: https://software.intel.com/sites/default/files/managed/d7/a9/realsense-camera-r200-product-datasheet.pdf

Installation of librealsense
----------------------------

- Ubuntu 16.04 and 18.04

Intel® RealSense™ SDK 2.0 can be installed with apt for ubuntu 16.04 and 18.04.
Please see `this page <https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md>`_ for more details.

.. code-block:: bash

  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) stable"
  sudo apt update
  sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

- Ubuntu 14.04

librealsense 2.0 or above is not distributed with debian package for ubuntu 14.04. so you have to build librealsense from source.
In addition, librealsense works stably on 4.4.xx kernels. So you need to upgrade your ubuntu kernel
Please see `this page <https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md>`_ for more details about manual installation.

.. code-block:: bash

  sudo apt-get install ros-indigo-realsense-camera

  cd ~
  git clone https://github.com/IntelRealSense/librealsense.git
  cd librealsense
  git checkout v0.9.2

  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger

  # Requirement Installation
  # If you already installed, you can skip here.

  ## gcc-4.9 and g++-4.9
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update
  sudo apt-get install gcc-4.9 g++-4.9
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9

  ## openssl
  sudo apt-get install libssl-dev

  # uvcvideo patch Installation
  ./scripts/patch-uvcvideo-4.4.sh v4.4-wily
  # this script sometimes causes error below with Ubuntu 14.04
  #
  # cp: will not overwrite just-created ‘./.config’ with ‘/usr/src/linux-headers-4.4.4-040404-generic/.config’
  #
  # If you got this error, see https://github.com/IntelRealSense/librealsense/issues/146
  # or see https://github.com/IntelRealSense/librealsense/issues/70
  #
  # my solution is https://gist.github.com/knorth55/8e76494a694a287a8cf00b54c38e29ad

  sudo modprobe uvcvideo
  # if you get error below, patch script is not successful.
  #
  # modprobe: ERROR: could not insert 'uvcvideo'


Installation of realsense-ros
-----

- Ubuntu 16.04 and 18.04

realsense-ros is available as a debian package. please see `this page <https://github.com/IntelRealSense/realsense-ros>`_ for more details.

.. code-block:: bash

  sudo apt install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description


- Ubuntu 14.04

.. code-block:: bash

  mkdir -p ~/catkin_ws/src
  cd catkin_ws/src
  git clone https://github.com/intel-ros/realsense.git
  git checkout 1.5.0
  cd ../..
  rosdep install --skip-keys=librealsense --ignore-src --from-path -i src -y -r
  catkin build

Sample Launch
-------------

- for D400 Series

.. code-block:: bash

    roslaunch realsense2_camera rs_camera.launch

- for T265

.. code-block:: bash

    roslaunch realsense2_camera rs_t265.launch

- for legacy version

.. code-block:: bash

  source ~/catkin_ws/devel/setup.bash
  # for SR300
  roslaunch realsense_camera sr300_nodelet_rgbd.launch
  # for R200
  roslaunch realsense_camera r200_nodelet_rgbd.launch

  # another terminal
  rosrun rviz rviz


Video
-----

- `SR300`_

- `R200`_

.. _SR300: https://drive.google.com/a/jsk.imi.i.u-tokyo.ac.jp/file/d/0B5DV6gwLHtyJU2REemx2OVNKY0U/view 

.. _R200: https://drive.google.com/a/jsk.imi.i.u-tokyo.ac.jp/file/d/0B5DV6gwLHtyJTG4yTzZ0UzZQTjA/view
