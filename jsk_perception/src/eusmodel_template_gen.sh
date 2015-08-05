#!/bin/bash -x

DEVEL_SHARE_DIR=$1
if [ ! -e "$DEVEL_SHARE_DIR" ]; then
    echo "DEVEL_SHARE_DIR not found. aborting..."
    return -1
fi

DEVEL_TEMPLATE_DIR=$DEVEL_SHARE_DIR/template

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

if [ -x $ROSEUS ]; then
    $ROSEUS ./src/eusmodel_template_gen.l
else
    echo "ROSEUS binary not found"
    return -1
fi
(cd sample; $ROSEUS ./pose_detector_auto_gen_sample.l "(progn (generate-launcher) (exit))")

mv -f template $DEVEL_TEMPLATE_DIR
