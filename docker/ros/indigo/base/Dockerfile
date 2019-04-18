FROM ros:indigo-ros-core

RUN apt-get update && apt-get install -y \
    ccache \
    git \
    wget \
    python-setuptools \
    python-rosdep \
    python-wstool

RUN ccache -M 10G
ENV PATH /usr/lib/ccache:$PATH

# Install pip
# pip>=10 no longer uninstalls distutils packages (ex. packages installed via apt),
# and fails to install packages via pip if they are already installed via apt.
# See https://github.com/pypa/pip/issues/4805 for detail.
RUN curl https://bootstrap.pypa.io/get-pip.py | python - \
    pip install 'pip<10'

RUN mkdir -p ~/ros/ws_jsk_recognition/src && \
    cd ~/ros/ws_jsk_recognition/src && \
    wstool init; \
    wstool set jsk-ros-pkg/jsk_recognition https://github.com/jsk-ros-pkg/jsk_recognition.git -v master --git -y && \
    wstool up  && \
    wstool merge -y jsk-ros-pkg/jsk_recognition/.travis.rosinstall; \
    wstool up -j -1

# for pip install numpy matplotlib pillow
RUN apt-get install -y build-essential pkg-config && \
    apt-get install -y liblapack-dev gfortran && \
    apt-get install -y libtiff5-dev libpng12-dev libjpeg8-dev zlib1g-dev \
      libfreetype6-dev liblcms2-dev libwebp-dev libharfbuzz-dev libfribidi-dev \
      tcl8.6-dev tk8.6-dev python-tk

RUN cd ~/ros/ws_jsk_recognition/src && \
    rosdep update && \
    wget http://raw.github.com/jsk-ros-pkg/jsk_travis/master/rosdep-install.sh -O - | bash

RUN cd ~/ros/ws_jsk_recognition && \
    apt-get install -y python-catkin-tools && \
    . /opt/ros/indigo/setup.sh && \
    catkin build -c && \
    cd && \
    rm -rf ~/ros
