Install Astra camera
====================

- Homepage: https://orbbec3d.com/

Spec
----

+------------------+------------------+------------------+------------------+------------------+------------------+
| Spec/Device      | Astra            | Astra Mini       | Astra S          | Astra Mini S     | Astra Pro        |
+==================+==================+==================+==================+==================+==================+
| Optimized Range  | 0.6 - 5.0m       | 0.6 - 5.0m       | 0.4 - 2.0m       | 0.35 - 1.0m      | 0.6 - 5.0m       |
+------------------+------------------+------------------+------------------+------------------+------------------+
| RGB Image Size   | | HD   (10FPS)   | | HD   (10FPS)   | | HD   (10FPS)   | | HD   (10FPS)   | | HD   (30FPS)   |
|                  | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   |
|                  | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   |
+------------------+------------------+------------------+------------------+------------------+------------------+
| Depth Image Size | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   | | VGA  (30FPS)   |
|                  | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   | | QVGA (30FPS)   |
+------------------+------------------+------------------+------------------+------------------+------------------+
| Field of View    | | 60°   horiz    |                  | | 60°   horiz    |                  | | 60°   horiz    |
|                  | | 49.5° vert     |                  | | 49.5° vert     |                  | | 49.5° vert     |
|                  | | 73°   diagonal |                  | | 73°   diagonal |                  | | 73°   diagonal |
+------------------+------------------+------------------+------------------+------------------+------------------+

Install SDK && Try Sample
-------------------------

.. code-block:: bash

  cd ~/Downloads
  # this can be broken
  # wget "https://www.dropbox.com/sh/p2uowlt3swdrfno/AACfEbv7ejIU-4FHy4Fyi0ZWa?dl=1" -O Tools_SDK_OpenNI.zip
  sudo pip install gdown
  gdown "https://drive.google.com/uc?id=0B9P1L--7Wd2vSktrZXFYMEZOWXM" -O Tools_SDK_OpenNI.zip

  mkdir ~/Downloads/Tools_SDK_OpenNI
  cd ~/Downloads/Tools_SDK_OpenNI

  unzip ~/Downloads/Tools_SDK_OpenNI.zip

  cd 2-Linux
  tar zxvf OpenNI-Linux-x64-2.2-0118.tgz
  cd OpenNI-Linux-x64-2.2

  sudo ./install.sh

  cd ~/Downloads/Tools_SDK_OpenNI/2-Linux/OpenNI-Linux-x64-2.2/Samples/Bin
  ./SimpleViewer  # This should open a viewer for depth image


Use Astra camera with ``openni2_camera`` ROS package
----------------------------------------------------

.. code-block:: bash

  sudo apt-get install ros-$ROS_DISTRO-openni2-camera ros-$ROS_DISTRO-openni2-launch

  cd ~/Downloads/Tools_SDK_OpenNI/2-Linux/OpenNI-Linux-x64-2.2/Samples/Bin

  sudo cp libOpenNI2.so /usr/lib/libOpenNI2.so
  sudo cp OpenNI2/Drivers/* /usr/lib/OpenNI2/Drivers/


Then, edit ``/usr/lib/pkgconfig/libopenni2.pc`` to be like below::

  prefix=/usr
  exec_prefix=${prefix}
  libdir=${exec_prefix}/lib
  includedir=${prefix}/include/openni2

  Name: OpenNI2
  Description: A general purpose driver for all OpenNI cameras.
  Version: 2.2.0.3
  Cflags: -I${includedir}
  Libs: -L${libdir} -lOpenNI2 -L${libdir}/OpenNI2/Drivers -lDummyDevice -lOniFile -lORBBEC -lPS1080 -lPSLink


.. code-block:: bash

  cd <your catkin workspace>/src
  # if you do not initialize wstools
  # wstool init .
  wstool set ros-drivers/openni2_camera https://github.com/ros-drivers/openni2_camera.git --git -v indigo-devel -y -u

  cd ros-drivers/openni2_camera
  source /opt/ros/$ROS_DISTRO/setup.bash
  catkin bt

  source <your catkin workspace>/devel/setup.bash
  roslaunch openni2_launch openni2.launch


Topics per Devices
------------------

Below are tested with ``roslaunch openni2_launch openni2.launch``.

+------------------------------------+-------+---------+-----------+--------------+
| Topics/Device                      | Astra | Astra S | Astra Pro | Astra Mini S |
+====================================+=======+=========+===========+==============+
| /camera/rgb/image_raw              | o     | o       | ?         | o            |
+------------------------------------+-------+---------+-----------+--------------+
| /camera/depth/image_raw            | o     | o       | ?         | o            |
+------------------------------------+-------+---------+-----------+--------------+
| /camera/depth/points               | o     | o       | ?         | o            |
+------------------------------------+-------+---------+-----------+--------------+
| /camera/depth_registered/image_raw | o     | o       | ?         | o            |
+------------------------------------+-------+---------+-----------+--------------+
| /camera/depth_registered/points    | x     | o       | ?         | o            |
+------------------------------------+-------+---------+-----------+--------------+
