FROM ros:indigo-ros-core

RUN apt-get update && apt-get install -y \
    git \
    python-setuptools \
    python-rosdep \
    python-wstool

RUN easy_install -U pip && \
    pip install -U pip setuptools

RUN mkdir -p ~/ros/ws_jsk_recognition/src && \
    cd ~/ros/ws_jsk_recognition/src && \
    wstool init; \
    wstool set jsk-ros-pkg/jsk_recognition https://github.com/jsk-ros-pkg/jsk_recognition.git -v master --git -y && \
    wstool up  && \
    wstool merge -y jsk-ros-pkg/jsk_recognition/.travis.rosinstall; \
    wstool up -j -1

RUN cd ~/ros/ws_jsk_recognition/src && \
    rosdep update && \
    rosdep install --from-path . -y -i

RUN cd ~/ros/ws_jsk_recognition && \
    apt-get install -y python-catkin-tools && \
    . /opt/ros/indigo/setup.sh && \
    catkin build && \
    catkin clean -y
