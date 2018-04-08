FROM ros-indigo-base-jsk-recognition

RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    libvtk5-dev

RUN cd ~ && \
    wget -q https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0rc2.tar.gz && \
    tar zxf pcl-1.8.0rc2.tar.gz

RUN cd ~/pcl-pcl-1.8.0rc2 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j2 && \
    make install

RUN mkdir -p ~/ros/ws_jsk_recognition/src && \
    cd ~/ros/ws_jsk_recognition/src && \
    apt-get install -y python-rosinstall-generator && \
    wstool init; \
    rosinstall_generator --tar --rosdistro indigo \
        pcl_conversions \
        pcl_ros \
        octomap_server \
        > $$.rosinstall && \
    wstool merge -y $$.rosinstall && \
    wstool up -j -1

RUN cd ~/ros/ws_jsk_recognition/src && \
    rosdep update && \
    wget http://raw.github.com/jsk-ros-pkg/jsk_travis/master/rosdep-install.sh -O - | bash

RUN cd ~/ros/ws_jsk_recognition && \
    . /opt/ros/indigo/setup.sh && \
    export PATH=/usr/lib/ccache:$PATH && \
    catkin build -c && \
    cd && \
    rm -rf ros/ws_jsk_recognition
