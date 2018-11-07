Install Ensenso
===============

- Homepage: https://www.ensenso.com/
- Online Manual: https://www.ensenso.com/manual/index.html  


Install SDK && Demo
-------------------

.. code-block:: bash

  cd ~/Downloads

  # install codemeter
  wget -O codemeter_6.40.2402.501_amd64.deb https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
  sudo dpkg -i codemeter_6.40.2402.501_amd64.deb

  # install ensenso sdk
  wget -O ensenso-sdk-2.0.147-x64.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.0.147-x64.deb
  sudo dpkg -i ensenso-sdk-2.0.147-x64.deb

  # install ueye_driver
  wget -O uEye_4.81.1_Linux_64.tgz https://download.ensenso.com/s/idsdrivers/download?files=uEye_4.81.1_Linux_64.tgz
  tar czf uEye_4.81.1_Linux_64.tgz
  sudo bash ./ueyesdk-setup-4.81.01-eth-amd64.gz.run
  sudo bash ./ueyesdk-setup-4.81.01-usb-amd64.gz.run
  sudo service ueyeethdrc start
  sudo service ueyeusbdrc start

  # demo
  nxView


Use Ensenso Camera with ``ensenso`` ROS Pacakage
------------------------------------------------

.. code-block:: bash

  mkdir catkin_ws/src -p
  cd catkin_ws/src
  git clone https://github.com/crigroup/ensenso.git
  cd ..
  catkin build
  roslaunch ensenso viewer.launch serial:=CAMERA_SERIAL rqt:=false
  rosrun tf static_transform_publisher 0 0 0 0 0 0 base camera_optical_frame
  rviz
