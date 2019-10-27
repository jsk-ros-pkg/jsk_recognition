Install Structure core Camera
=============================

- Homepage: https://structure.io/structure-core

Install SDK && Try Sample
-------------------------

- Join the Developer Program and Download SDK (Cross-Platform)
- https://developer.structure.io/sdk

.. code-block:: bash

  # Download SDK to ~/Downloads
  cd ~/Downloads
  unzip StructureSDK-CrossPlatform-0.7.2.zip
  cd StructureSDK-CrossPlatform-0.7.2-ROS\ Driver\ beta/

  # Create udev rules for non root users
  chmod +x ./DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh
  sudo ./DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh 

  # Update firmware of the sensor
  chmod +x ./DriverAndFirmware/Linux/CoreFirmwareUpdater-0.9.7-Linux-x86_64 
  sudo ./DriverAndFirmware/Linux/CoreFirmwareUpdater-0.9.7-Linux-x86_64 

  # Build the SDK
  chmod +x ./Scripts/build.sh
  ./Scripts/build.sh

  # Run the examples
  ./Builds/linux-release-x86_64/Samples/CorePlayground/CorePlayground


Install ROS Driver beta
-----------------------

.. code-block:: bash

  # Move SDK Repository to catkin workspace
  cd ~/Downloads
  mv StructureSDK-CrossPlatform-0.7.2-ROS\ Driver\ beta/ ~/catkin_ws/src/
  cd catkin_ws/src/StructureSDK-CrossPlatform-0.7.2-ROS\ Driver\ beta/ROS

  # changed permission
  chmod +x ros1_driver/cfg/SCParams.cfg 

  # source to_ros1.sh script and build the package
  chmod +x to_ros1.sh
  source to_ros1.sh
  catkin build

  # launch 
  source ~/catkin_ws/devel/setup.bash
  roslaunch structure_core_ros_driver sc.launch
