FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu14.04
LABEL maintainer "iory <ab.ioryz@gmail.com>"

ENV ROS_DISTRO indigo

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt update && apt install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ros-core \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    python-pip && \
    pip install pip==9.0.3

# chainer install
RUN pip install --no-cache-dir cupy-cuda92==5.1.0 chainer==5.1.0

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install jsk_recognition
RUN mkdir -p /ros_ws/src && \
    cd /ros_ws/src && \
    git clone --depth 1 https://github.com/jsk-ros-pkg/jsk_recognition

# build ros package
RUN cd /ros_ws; rosdep install -r -y --from-paths src --ignore-src
RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; cd /ros_ws; catkin build jsk_recognition -j4
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

RUN rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

CMD ["bash"]
