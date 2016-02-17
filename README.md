jsk\_recognition
===============

[![Build Status](https://travis-ci.org/jsk-ros-pkg/jsk_recognition.svg)](https://travis-ci.org/jsk-ros-pkg/jsk_recognition)
[![Read the Docs](https://readthedocs.org/projects/jsk-docs/badge/?version=latest)](http://jsk-docs.readthedocs.org/en/latest/jsk_recognition/doc/index.html)

Deb Build Status
------------

Package | Indigo (Saucy) | Indigo (Trusty) |
------- | -------------- | --------------- |
jsk_recognition (32-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-jsk-recognition_binarydeb_saucy_i386)](http://jenkins.ros.org/job/ros-indigo-jsk-recognition_binarydeb_saucy_i386/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-jsk-recognition_binarydeb_trusty_i386)](http://jenkins.ros.org/job/ros-indigo-jsk-recognition_binarydeb_trusty_i386/)
jsk_recognition (64-bit) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-jsk-recognition_binarydeb_saucy_amd64)](http://jenkins.ros.org/job/ros-indigo-jsk-recognition_binarydeb_saucy_amd64/) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-jsk-recognition_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-jsk-recognition_binarydeb_trusty_amd64/)


Documentation for jsk\_recognition
----------------------------------
You will find documentation about nodes and types of jsk\_recognition at [the Read the Docs site](https://jsk-recognition.readthedocs.org).


jsk_recognition is a stack for the perception packages which are used in JSK lab.

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
