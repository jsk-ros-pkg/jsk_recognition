FROM ros:kinetic-ros-core

RUN apt update && apt install -y \
    wget \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    libvtk5-dev \
    python-catkin-tools \
    python-rosinstall-generator \
    python-wstool
RUN easy_install 'pip==6.0.7'
RUN pip install -U pip==9.0.3
RUN pip install -U dlib

# VTK install
RUN cd ~ && \
    wget -q http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz && \
    tar -xf VTK-7.1.0.tar.gz && \
    cd ~/VTK-7.1.0 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j2 && \
    make install

# PCL install
RUN cd ~ && \
    wget -q https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0rc2.tar.gz && \
    tar zxf pcl-1.8.0rc2.tar.gz && \
    cd ~/pcl-pcl-1.8.0rc2 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DPCL_ENABLE_SSE:BOOL=FALSE .. && \
    make -j2 && \
    make install

RUN mkdir -p ~/ros/ws_jsk_recognition/src && \
    cd ~/ros/ws_jsk_recognition/src && \
    rosinstall_generator --tar --rosdistro ${ROS_DISTRO} \
        pcl_conversions \
        pcl_ros \
        octomap_server \
        > .rosinstall && \
    wstool up -j -1

RUN cd ~/ros/ws_jsk_recognition/src && \
    git clone --depth 1 https://github.com/jsk-ros-pkg/jsk_recognition && \
    cd ~/ros/ws_jsk_recognition && \
    rosdep update && \
    rosdep install --from-paths --ignore-src -y -r src && \
    rm -rf /var/lib/apt/lists/*

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; cd ~/ros/ws_jsk_recognition; catkin build -j4
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

CMD ["bash"]
