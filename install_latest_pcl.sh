#!/bin/bash

set -e
function redecho()
{
    echo -e "\e[1;31m" $@ "\e[m"
}

if [ "$1" == "" ]; then
    redecho "Please specify a directory to install"
    exit 1
fi

# mkdir
redecho "[mkdir $1]"
mkdir -p $1

BOOST_URL="http://downloads.sourceforge.net/project/boost/boost/1.48.0/boost_1_48_0.tar.gz?r=http%3A%2F%2Fsourceforge.net%2Fprojects%2Fboost%2Ffiles%2Fboost%2F1.48.0%2F&ts=1409712113&use_mirror=jaist"
ROOT_DIR=$1
BOOST_DIR=$1/boost
BOOST_SOURCE_DIR=$BOOST_DIR/boost_1_48_0
BOOST_INSTALL_DIR=$1/usr
PCL_SOURCE_DIR=$1/pcl
PCL_BUILD_DIR=$PCL_SOURCE_DIR/build
PCL_INSTALL_DIR=$1/usr
PCL_URL=https://github.com/PointCloudLibrary/pcl.git
ROS_SRC_DIR=$1/ros_parent/src
ROS_CHILD_SRC_DIR=$1/ros/src
########################################################
## boost
########################################################

redecho "installing boost 1.48"
mkdir -p $BOOST_DIR
if [ ! -e $BOOST_DIR/boost.tar.gz ]; then
    redecho downloading boost
    (cd $BOOST_DIR && wget $BOOST_URL -O boost.tar.gz)
fi

if [ ! -e $BOOST_SOURCE_DIR ]; then
    redecho expanding boost
    (cd $BOOST_DIR && tar xvzf boost.tar.gz)
fi

if [ ! -e $BOOST_SOURCE_DIR/b2 ]; then
    redecho building boost
    (cd $BOOST_SOURCE_DIR && ./bootstrap.sh --prefix=$BOOST_INSTALL_DIR)
    (cd $BOOST_SOURCE_DIR && ./b2 install -j$(grep -c processor /proc/cpuinfo) link=static,shared)
fi

########################################################
## pcl
########################################################
redecho installing pcl
if [ ! -e $PCL_SOURCE_DIR ]; then
    redecho cloning pcl
    git clone $PCL_URL $PCL_SOURCE_DIR
fi

if [ ! -e $PCL_BUILD_DIR ]; then
    redecho building pcl
    mkdir -p $PCL_BUILD_DIR
    # patching for haswell...
    (cd $PCL_SOURCE_DIR/cmake && sed s/-march=native/-march=corei7/g -i pcl_find_sse.cmake)
    (cd $PCL_BUILD_DIR && BOOST_ROOT=$BOOST_INSTALL_DIR cmake .. -DBoost_REALPATH=ON -DBoost_NO_SYSTEM_PATHS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=$PCL_INSTALL_DIR && make -j$(grep -c processor /proc/cpuinfo) && make install)

fi

########################################################
## ROS
########################################################
mkdir -p $ROS_SRC_DIR
if [ ! -e $ROS_SRC_DIR/CMakeLists.txt ]; then
    redecho initalize catkin workspace
    (cd $ROS_SRC_DIR && catkin_init_workspace)
fi
if [ ! -e $ROS_SRC_DIR/.rosinstall ]; then
    redecho inisitalize wstool
    (cd $ROS_SRC_DIR && wstool init)
fi

redecho compiling ros parent workspace
(cd $ROS_SRC_DIR && rosinstall_generator jsk_pcl_ros jsk_perception --flat --deps --deps-only --wet-only --tar --exclude euslisp roseus| wstool merge -)
(cd $ROS_SRC_DIR && wstool update -j10)

unset CMAKE_PREFIX_PATH

(cd $ROS_SRC_DIR/.. &&  BOOST_ROOT=$BOOST_INSTALL_DIR catkin_make_isolated --install -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBoost_REALPATH=ON -DBoost_NO_SYSTEM_PATHS=ON -DBOOST_ROOT=$BOOST_INSTALL_DIR -DPCL_DIR=$PCL_INSTALL_DIR/share/pcl-1.7)

redecho compiling ros child workspace
mkdir -p $ROS_CHILD_SRC_DIR
if [ ! -e $ROS_CHILD_SRC_DIR/CMakeLists.txt ]; then
    redecho initalize catkin workspace
    (cd $ROS_CHILD_SRC_DIR && catkin_init_workspace)
fi
if [ ! -e $ROS_CHILD_SRC_DIR/.rosinstall ]; then
    redecho inisitalize wstool
    (cd $ROS_CHILD_SRC_DIR && wstool init)
fi

(cd $ROS_CHILD_SRC_DIR && rosinstall_generator jsk_pcl_ros --wet-only --tar --upstream-development| wstool merge -)
(cd $ROS_CHILD_SRC_DIR && wstool update -j10)
unset CMAKE_PREFIX_PATH
source $ROS_SRC_DIR/../install_isolated/setup.sh
(cd $ROS_CHILD_SRC_DIR/.. &&  BOOST_ROOT=$BOOST_INSTALL_DIR catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBoost_REALPATH=ON -DBoost_NO_SYSTEM_PATHS=ON -DBOOST_ROOT=$BOOST_INSTALL_DIR -DPCL_DIR=$PCL_INSTALL_DIR/share/pcl-1.7 --only-pkg-with-deps jsk_pcl_ros)

redecho done, please check out $ROS_CHILD_SRC_DIR and $ROS_CHILD_SRC_DIR/../devel directory
