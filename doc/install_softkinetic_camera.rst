Install Softkinetic Camera
==========================


Devices
-------

This instruction supports below cameras:

- Softkinetic
- DepthSense DS325


Installation
------------

1. Install DepthSenseSDK. You need to login to download it.  (`Middleware -> Download <http://www.softkinetic.com/>`_)
2. Install ROS package::

  cd ~/ros/indigo/src
  wstool set ipa320/softkinetic https://github.com/ipa320/softkinetic.git --git -vindigo_dev -y --update
  cd softkinetic/softkinetic && catkin bt -iv

3. Connect device and run::

  roslaunch softkinetic_camera softkinetic_camera_demo.launch


Known Issues
------------

Error: No space left on device
+++++++++++++++++++++++++++++++
::

  [ INFO] [1432824767.747549238]: Number of Devices found: 1
  Argument Exception: argument "node": context already has control over the node
  terminate called after throwing an instance of 'DepthSense::StreamingException'
    what():  VIDIOC_STREAMON failed (No space left on device)

If you have above error, you can try below::

  vim /etc/modprobe.d/blacklist.conf
  # add 'blacklist snd_usb_audio' at the bottom

**References**

- https://github.com/ipa320/softkinetic/issues/48
- http://www.softkinetic.com/support/Forum/aft/1735
