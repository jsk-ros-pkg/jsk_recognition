#!/bin/bash -x

cd `rospack find jsk_perception`
mkdir -p launch
mkdir -p template
#export ROS_MASTER_URI=http://localhost:12347
#roscore -p 12347 & (roseus ./src/eusmodel_template_gen.l && kill -INT $!)
ROSEUS_BIN_PATH=$(catkin_find --without-underlays --libexec --share roseus)
for bin_path in $(echo $ROSEUS_BIN_PATH)
do
    if [ -e $bin_path/bin/roseus ]; then
        ROSEUS=$bin_path/bin/roseus
    elif [ -e $bin_path/roseus ]; then
        ROSEUS=$bin_path/roseus
    fi
done

$ROSEUS ./src/eusmodel_template_gen.l
(cd sample; $ROSEUS ./pose_detector_auto_gen_sample.l "(progn (generate-launcher) (exit))")
