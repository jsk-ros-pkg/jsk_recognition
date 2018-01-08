FROM ros:indigo-ros-core

RUN apt-get update && apt-get install -y \
    wget \
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
    apt-get install -y python-rosinstall-generator python-wstool && \
    rosinstall_generator --tar --rosdistro indigo \
        pcl_conversions \
        pcl_ros \
        octomap_server \
        > .rosinstall && \
    wstool up -j -1

RUN cd ~/ros/ws_jsk_recognition/src && \
    rosdep update && \
    rosdep install --from-path . -y -i

RUN cd ~/ros/ws_jsk_recognition && \
    apt-get install -y python-catkin-tools && \
    . /opt/ros/indigo/setup.sh && \
    catkin build

RUN apt-get update && apt-get install -y python-setuptools
RUN easy_install 'pip==6.0.7'
RUN pip install -U dlib
