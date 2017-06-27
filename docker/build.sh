#!/bin/bash

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ros-indigo-pcl1.8
(cd ${DIR}/ros/indigo/pcl1.8 && sudo docker build -t ros-indigo-pcl1.8 .)
(echo -e "FROM ros-indigo-pcl1.8\nRUN apt-get update\nRUN apt-get -y upgrade\nEXPOSE 22" | sudo docker build -t ros-indigo-pcl1.8 -)
