Install RealSense camera
====================

- librealsense: https://github.com/IntelRealSense/librealsense

- realsense-ros: https://github.com/intel-ros/realsense

- Intel realsense robotic development kit: https://01.org/developerjourney/recipe/intel-realsense-robotic-development-kit

- SR300 Data Sheet: https://software.intel.com/sites/default/files/managed/0c/ec/realsense-sr300-product-datasheet-rev-1-0.pdf

- R200 Data Sheet: https://software.intel.com/sites/default/files/managed/d7/a9/realsense-camera-r200-product-datasheet.pdf

Kernel Update
-------------

librealsense works stably on 4.4.xx kernels.

This installation guide is only appropriate for 4.4.xx kernels


RealSense-ROS Installation
--------------------------

.. code-block:: bash

  sudo apt-get install ros-indigo-realsense-camera

  cd ~
  git clone https://github.com/IntelRealSense/librealsense.git
  cd librealsense
  git checkout v0.9.2

  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger

  ./scripts/install_dependencies-4.4.sh
  ./scripts/patch-uvcvideo-4.4.sh v4.4-wily

  sudo modprobe uvcvideo

Build
-----

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
