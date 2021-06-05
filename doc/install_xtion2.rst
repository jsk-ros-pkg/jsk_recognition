Install Xtion2
==============

- Homepage: https://www.asus.com/3D-Sensor/Xtion-2/
- SDK Download: https://www.asus.com/3D-Sensor/Xtion-2/HelpDesk_Download/

Spec
----

+------------------+------------------+
| Spec/Device      | Xtion2           |
+==================+==================+
| Optimized Range  | 0.8 - 3.5m       |
+------------------+------------------+
| RGB Image Size   | 5MP (2592x1944)  |
+------------------+------------------+
| Depth Image Size | | VGA  (30FPS)   |
|                  | | QVGA (30FPS)   |
+------------------+------------------+
| Field of View    | | 74°   horiz    |
|                  | | 52°   vert     |
|                  | | 90°   diagonal |
+------------------+------------------+

Install SDK && Try Sample
-------------------------

.. code-block:: bash

  # Download SDK.zip from https://www.asus.com/3D-Sensor/Xtion-2/HelpDesk_Download/
  cd ~/Downloads
  tar xf ASUS-Linux-x64-OpenNI2.2.tar.gz
  cd ASUS-Linux-x64-OpenNI2.2/

  sudo apt-get install ros-$ROS_DISTRO-openni2-launch ros-$ROS_DISTRO-openni2-camera libopenni2-dev
  sudo ./install.sh
  # you need to copy libSenDuck.so by manual
  sudo cp ASUS/Xtion2/lib/libSenDuck.so /usr/lib/OpenNI2/Drivers/

  cd Samples/Bin
  ./SimpleViewer  # This should open a viewer for depth image


Use Xtion2 camera with ``openni2_camera`` ROS package
-----------------------------------------------------

.. code-block:: bash

  cd ~/ros/$ROS_DISTRO/src
  wstool set ros-drivers/openni2_camera https://github.com/ros-drivers/openni2_camera.git --git -v 0.2.8 -y -u

  source /opt/ros/$ROS_DISTRO/setup.bash
  catkin b

  source ~/ros/$ROS_DISTRO/devel/setup.bash
  roslaunch openni2_launch openni2.launch
