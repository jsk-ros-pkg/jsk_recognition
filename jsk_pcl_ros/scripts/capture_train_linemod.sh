#!/bin/sh

if [ "$1" != "" ]; then
    BAG_FILE="$1"
    else
    BAG_FILE=/home/lueda/multisense_capture_2014-12-30-13-40-27.bag
fi

rosbag play $BAG_FILE -r 10
rosservice call /incremental_model_registration/start_registration
rosservice call /linemod_trainer/start_training
