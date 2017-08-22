#!/bin/bash

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set -x

(cd ${DIR}/ros/indigo/base && sudo docker build --no-cache -t ros-indigo-base-jsk-recognition .)

(cd ${DIR}/ros/indigo/pcl1.8 && sudo docker build --no-cache -t ros-indigo-pcl1.8-jsk-recognition .)

set +x
