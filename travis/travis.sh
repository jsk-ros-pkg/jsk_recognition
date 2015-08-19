#!/bin/bash

set -x

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}


if [ "$ROS_DISTRO" == "indigo" -o "$ROS_DISTRO" == "jade" -o "${USE_JENKINS}" == "true" ] && [ "$TRAVIS_JOB_ID" ]; then
    sudo apt-get install -y -qq python-pip
    pip install --user python-jenkins
    ./travis/travis_jenkins.py
    exit $?
fi

function error {
    travis_time_end 31
    trap - ERR
    exit 1
}

[ "$BUILDER" == rosbuild ] && ( echo "$BUILDER is no longer supported"; exit 1; )
[ "$ROSWS" == rosws ] && ( echo "$ROSWS is no longer supported"; exit 1; )
BUILDER=catkin
ROSWS=wstool

trap error ERR

travis_time_start setup_ros

# Define some config vars
export CI_SOURCE_PATH=$(pwd)
export REPOSITORY_NAME=${PWD##*/}
if [ ! "$ROS_PARALLEL_JOBS" ]; then export ROS_PARALLEL_JOBS="-j8";  fi
if [ ! "$CATKIN_PARALLEL_JOBS" ]; then export CATKIN_PARALLEL_JOBS="-p4";  fi
if [ ! "$ROS_PARALLEL_TEST_JOBS" ]; then export ROS_PARALLEL_TEST_JOBS="$ROS_PARALLEL_JOBS";  fi
if [ ! "$CATKIN_PARALLEL_TEST_JOBS" ]; then export CATKIN_PARALLEL_TEST_JOBS="$CATKIN_PARALLEL_JOBS";  fi
if [ ! "$ROS_REPOSITORY_PATH" ]; then export ROS_REPOSITORY_PATH="http://packages.ros.org/ros-shadow-fixed/ubuntu"; fi
echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo -E sh -c 'echo "deb $ROS_REPOSITORY_PATH `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
lsb_release -a
sudo apt-get update
sudo apt-get install -y -q -qq python-rosdep python-pip python-wstool python-catkin-tools ros-$ROS_DISTRO-rosbash ros-$ROS_DISTRO-rospack
if [ "$EXTRA_DEB" ]; then sudo apt-get install -q -qq -y $EXTRA_DEB;  fi
# MongoDB hack - I don't fully understand this but its for moveit_warehouse
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get remove -q -qq -y mongodb mongodb-10gen || echo "ok"; fi
if [ $HAVE_MONGO_DB == 0 ]; then sudo apt-get install -q -qq -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"; fi # default actions

travis_time_end
travis_time_start setup_rosdep

# Setup rosdep
sudo rosdep init
ret=1
rosdep update || while [ $ret != 0 ]; do sleep 1; rosdep update && ret=0 || echo "failed"; done

travis_time_end
travis_time_start setup_catkin

### before_install: # Use this to prepare the system to install prerequisites or dependencies
## to avoid stty error, until catkin_tools 2.0.x (http://stackoverflow.com/questions/27969057/cant-launch-catkin-build-from-jenkins-job)
sudo apt-get install -q -qq -y python-setuptools python-catkin-pkg
[ ! -e /tmp/catkin_tools ] && (cd /tmp/; git clone -q https://github.com/catkin/catkin_tools)
(cd /tmp/catkin_tools; sudo python setup.py --quiet install)
### https://github.com/ros/catkin/pull/705
[ ! -e /tmp/catkin ] && (cd /tmp/; git clone -q https://github.com/ros/catkin)
(cd /tmp/catkin; cmake . -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO/ ; make; sudo make install)
sudo apt-get install -y -q -qq ros-$ROS_DISTRO-roslaunch
### https://github.com/ros/ros_comm/pull/641
(cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/641.diff -O /tmp/641.diff; [ "$ROS_DISTRO" == "hydro" ] && sed -i s@items@iteritems@ /tmp/641.diff ; sudo patch -p4 < /tmp/641.diff)


travis_time_end
travis_time_start setup_rosws

### install: # Use this to install any prerequisites or dependencies necessary to run your build
# Create workspace
mkdir -p ~/ros/ws_$REPOSITORY_NAME/src
cd ~/ros/ws_$REPOSITORY_NAME/src
if [ "$USE_DEB" == false ]; then $ROSWS init .   ; fi
if [ "$USE_DEB" == false -a -e $CI_SOURCE_PATH/.rosinstall ]; then $ROSWS merge file://$CI_SOURCE_PATH/.rosinstall      ; fi
if [ "$USE_DEB" == false -a -e $CI_SOURCE_PATH/.rosinstall ]; then sed -i "s@^\(.*github.com/$TRAVIS_REPO_SLUG.*\)@#\1@" .rosinstall               ; fi # comment out current repo
if [ "$USE_DEB" == false ]; then $ROSWS update   ; fi
if [ "$USE_DEB" == false ]; then $ROSWS set $REPOSITORY_NAME http://github.com/$TRAVIS_REPO_SLUG --git -y        ; fi
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace
if [ "$USE_DEB" == source -a -e $REPOSITORY_NAME/setup_upstream.sh ]; then $ROSWS init .; $REPOSITORY_NAME/setup_upstream.sh -w ~/ros/ws_$REPOSITORY_NAME ; $ROSWS update; fi
# disable hrpsys/doc generation
find . -ipath "*/hrpsys/CMakeLists.txt" -exec sed -i s'@if(ENABLE_DOXYGEN)@if(0)@' {} \;

# Install dependencies for source repos
if [ "$ROSDEP_UPDATE_QUIET" == "true" ]; then
    ROSDEP_ARGS=>/dev/null
fi
source /opt/ros/$ROS_DISTRO/setup.bash # ROS_PACKAGE_PATH is important for rosdep

if [ ! -e .rosinstall ]; then
    echo "- git: {local-name: $REPOSITORY_NAME, uri: 'http://github.com/$TRAVIS_REPO_SLUG'}" >> .rosinstall
fi

travis_time_end

travis_time_start before_script

### before_script: # Use this to prepare your build for testing e.g. copy database configurations, environment variables, etc.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
if [ "$BEFORE_SCRIPT" != "" ]; then sh -c "${BEFORE_SCRIPT}"; fi

travis_time_end

travis_time_start rosdep_install

if [ -e ${CI_SOURCE_PATH}/.travis/rosdep-install.sh ]; then ## this is mainly for jsk_travis itself
    ${CI_SOURCE_PATH}/.travis/rosdep-install.sh
else
    wget http://raw.github.com/jsk-ros-pkg/jsk_travis/master/rosdep-install.sh -O - | bash
fi


travis_time_end

$ROSWS info -t .
cd ../

travis_time_start catkin_build

### script: # All commands must exit with code 0 on success. Anything else is considered failure.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
# for catkin
if [ "$TARGET_PKGS" == "" ]; then export TARGET_PKGS=`catkin_topological_order ${CI_SOURCE_PATH} --only-names`; fi
if [ "$TEST_PKGS" == "" ]; then export TEST_PKGS=$( [ "$BUILD_PKGS" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS"); fi
if [ "$BUILDER" == catkin ]; then catkin build -i -v --limit-status-rate 0.001 $BUILD_PKGS $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS            ; fi

travis_time_end
travis_time_start catkin_run_tests

if [ "$BUILDER" == catkin ]; then catkin run_tests --limit-status-rate 0.001 $TEST_PKGS $CATKIN_PARALLEL_TEST_JOBS --make-args $ROS_PARALLEL_TEST_JOBS --; fi
# it seems catkin run_tests write test result to wrong place, and ceate MISSING...
if [ "$BUILDER" == catkin ]; then  find build -iname MISSING* -print -exec rm {} \;; catkin_test_results build || error  ; fi

travis_time_end

if [ "$NOT_TEST_INSTALL" != "true" ]; then

    travis_time_start catkin_install_build

    if [ "$BUILDER" == catkin ]; then catkin clean -a                        ; fi
    if [ "$BUILDER" == catkin ]; then catkin config --install                ; fi
    if [ "$BUILDER" == catkin ]; then catkin build -i -v --limit-status-rate 0.001 $BUILD_PKGS $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS            ; fi
    if [ "$BUILDER" == catkin ]; then source install/setup.bash              ; fi
    if [ "$BUILDER" == catkin ]; then rospack profile                        ; fi
    if [ "$BUILDER" == catkin ]; then rospack plugins --attrib=plugin nodelet; fi

    travis_time_end
    travis_time_start catkin_install_run_tests

    if [ "$BUILDER" == catkin ]; then export EXIT_STATUS=0; for pkg in $TEST_PKGS; do echo "test $pkg..." ;[ "`find install/share/$pkg -iname '*.test'`" == "" ] && echo "[$pkg] No tests were found!!!"  || find install/share/$pkg -iname "*.test" -print0 | xargs -0 -n1 -I{} sh -c 'echo {}; rostest {}' || export EXIT_STATUS=$?; done; [ $EXIT_STATUS == 0 ] || error; fi

    travis_time_end

fi

travis_time_start after_script

## after_script
PATH=/usr/local/bin:$PATH  # for installed catkin_test_results
PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH
if [ "$ROS_LOG_DIR" == "" ]; then export ROS_LOG_DIR=~/.ros/test_results; fi # http://wiki.ros.org/ROS/EnvironmentVariables#ROS_LOG_DIR
if [ "$BUILDER" == catkin -a -e $ROS_LOG_DIR ]; then catkin_test_results --verbose --all $ROS_LOG_DIR || error; fi
if [ "$BUILDER" == catkin -a -e ~/ros/ws_$REPOSITORY_NAME/build/ ]; then catkin_test_results --verbose --all ~/ros/ws_$REPOSITORY_NAME/build/ || error; fi

travis_time_end
