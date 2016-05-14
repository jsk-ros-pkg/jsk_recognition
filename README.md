jsk\_recognition
===============

[![GitHub version](https://badge.fury.io/gh/jsk-ros-pkg%2Fjsk_recognition.svg)](https://badge.fury.io/gh/jsk-ros-pkg%2Fjsk_recognition)
[![Build Status](https://travis-ci.org/jsk-ros-pkg/jsk_recognition.svg)](https://travis-ci.org/jsk-ros-pkg/jsk_recognition)
[![Read the Docs](https://readthedocs.org/projects/jsk-docs/badge/?version=latest)](http://jsk-docs.readthedocs.org/en/latest/jsk_recognition/doc/index.html)

Deb Build Status
------------

| Package | Indigo (Saucy) | Indigo (Trusty) | Jade (Trusty) | Jade (Vivid) |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| jsk_recognition (32-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS32__jsk_recognition__ubuntu_saucy_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uS32__jsk_recognition__ubuntu_saucy_i386__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT32__jsk_recognition__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uT32__jsk_recognition__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT32__jsk_recognition__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uT32__jsk_recognition__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV32__jsk_recognition__ubuntu_vivid_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uV32__jsk_recognition__ubuntu_vivid_i386__binary/) |
| jsk_recognition (64-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS64__jsk_recognition__ubuntu_saucy_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uS64__jsk_recognition__ubuntu_saucy_amd64__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT64__jsk_recognition__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uT64__jsk_recognition__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT64__jsk_recognition__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uT64__jsk_recognition__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV64__jsk_recognition__ubuntu_vivid_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uV64__jsk_recognition__ubuntu_vivid_amd64__binary/) |

Documentation for jsk\_recognition
----------------------------------
You will find documentation about nodes and types of jsk\_recognition at [the Read the Docs site](https://jsk-recognition.readthedocs.org).


jsk_recognition is a stack for the perception packages which are used in JSK lab.


ROS packages
------------

### [checkerboard\_detector](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/checkerboard_detector)
ROS nodes to detect checkerboard. It supports chessboard pattern, circledots pattern and asymmetric
circledots pattern.

### [imagesift](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/imagesift)
ROS nodes to compute local features of 2-D images. It supports SIFT, SURF, STAR, BRISK.

### [jsk\_recognition\_msgs](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_recognition_msgs)
ROS messages for jsk\_pcl\_ros and jsk\_perception.

### [jsk\_recognition\_utils](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_recognition_utils)
C++ library about sensor model, geometrical modeling and perception.


### [jsk\_pcl\_ros](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_pcl_ros)
ROS nodelets for pointcloud perception.

### [jsk\_perception](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_perception)
ROS nodes and nodelets for 2-D image perception.

### [posedetectiondb](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/posedetectiondb)
ROS node to compute 3-D pose of 2-D texture based on local features.

### [resized\_image\_transport](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/resized_image_transport)
ROS nodes to publish resized images.

Deprecated packages
-------------------
* [cr\_calibration](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/cr_calibration)
* [cr\_capture](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/cr_capture)
* [orbit\_pantilt](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/orbit_pantilt)
