#!/bin/sh

cd ~/ros/ws_$REPOSITORY_NAME
rospack profile
if [ $BUILDER = catkin ]; then
    catkin_make -j4
    catkin_make test
    catkin_make install
    rm -fr devel src build
    source install/setup.bash
    find install -iname "*.test" -print0 | xargs -0 -n1 rostest
else
    cd src
    source setup.sh
    rosmake $BUILD_PACKAGES
fi
