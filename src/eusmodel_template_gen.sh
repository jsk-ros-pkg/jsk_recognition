#!/bin/bash -x

cd `rospack find jsk_perception`
mkdir -p launch
mkdir -p template
export ROS_MASTER_URI=http://localhost:12347
roscore & (roseus ./src/eusmodel_template_gen.l && kill -INT $!)
