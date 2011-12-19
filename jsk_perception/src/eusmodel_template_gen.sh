#!/bin/bash -x

cd `rospack find jsk_perception`
mkdir -p launch
mkdir -p template
#export ROS_MASTER_URI=http://localhost:12347
#roscore -p 12347 & (roseus ./src/eusmodel_template_gen.l && kill -INT $!)
rosrun roseus roseus ./src/eusmodel_template_gen.l
(cd sample; rosrun roseus roseus ./pose_detector_auto_gen_sample.l "(progn (generate-launcher) (exit))")
