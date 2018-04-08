FROM ros:indigo-ros-core

RUN apt-get update && apt-get install -y \
    ccache \
    git \
    python-setuptools \
    python-rosdep \
    python-wstool

RUN ccache -M 10G

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
    rosdep install --from-path . -y -i -r

RUN cd ~/ros/ws_jsk_recognition && \
    apt-get install -y python-catkin-tools && \
    . /opt/ros/indigo/setup.sh && \
    export PATH=/usr/lib/ccache:$PATH && \
    catkin build -c && \
    cd && \
    rm -rf ros/ws_jsk_recognition
