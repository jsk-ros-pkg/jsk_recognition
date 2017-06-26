#!/bin/bash

(cd ros/indigo/pcl1.8 && sudo docker build -t ros-indigo-pcl1.8 .)
(echo -e "FROM ros-ubuntu:14.04\nRUN apt-get update\nRUN apt-get -y upgrade\nEXPOSE 22" | sudo docker build -t ros-ubuntu:14.04 -)
