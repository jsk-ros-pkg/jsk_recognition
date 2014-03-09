#!/bin/sh

mkdir -p ~/ros/ws_$REPOSITORY_NAME/src
cd ~/ros/ws_$REPOSITORY_NAME/src
$WSTOOL init .
$WSTOOL merge http://raw.github.com/$TRAVIS_REPO_SLUG/$TRAVIS_BRANCH/.rosinstall
$WSTOOL update
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace
cd ../
# Install dependencies for source repos
rosdep install -r -n --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
