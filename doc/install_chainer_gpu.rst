Install Chainer with GPU Support
================================

This documentation describes how to install Chainer with GPU suppport.


Requirements
------------

- Nvidia GPU (ex. K80, TitanX, GTX 1080Ti).
- Ubuntu (ex. 14.04, 16.04).

  You can check whether your PC has a GPU by ``lspci | grep -i nvidia``.

Install CUDA
------------

- Download deb file from https://developer.nvidia.com/cuda-downloads?target_os=Linux::

    # If you'd like to use CUDA8.0 on Ubuntu 14.04.
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb
    mv cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo apt-get update
    sudo apt-get install cuda

    # If you'd like to use CUDA9.2 on Ubuntu 16.04.
    # Choose the green buttons on the web page like x86_64 -> Ubuntu -> version -> deb (network).
    # Excute 1-3 and then, change step 4 as follows:
    sudo apt install cuda-9-2

- After rebooting, you can see the memory usage of your GPU by ``nvidia-smi``

Install CUDNN
-------------

- You need to login at https://developer.nvidia.com/cudnn
- Go to cuDNN Download and choose version
- Download deb files of cuDNN Runtime Library and cuDNN Developer Library

::

   # If you'd like to install cuDNN for CUDA9.2 on Ubuntu 16.04
   # Download cuDNN v7.3.1 Runtime Library for Ubuntu16.04 (Deb)
   sudo dpkg -i libcudnn7_7.3.1.20-1+cuda9.2_amd64.deb
   # Download cuDNN v7.3.1 Developer Library for Ubuntu16.04 (Deb)
   sudo dpkg -i libcudnn7-dev_7.3.1.20-1+cuda9.2_amd64.deb

Install Chainer
---------------

::

    pip install chainer


Install Cupy
------------

- Add below to your `~/.bashrc`::

    # setup cuda & cudnn
    export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib:$LD_LIBRARY_PATH
    export LIBRARY_PATH=/usr/local/lib:/usr/lib:$LIBRARY_PATH
    export CPATH=/usr/include:$CPATH
    export CFLAGS=-I/usr/include
    export LDFLAGS="-L/usr/local/lib -L/usr/lib"
    if [ -e /usr/local/cuda ]; then
      export CUDA_PATH=/usr/local/cuda
      export PATH=$CUDA_PATH/bin:$PATH
      export CPATH=$CUDA_PATH/include:$CPATH
      export LD_LIBRARY_PATH=$CUDA_PATH/lib64:$CUDA_PATH/lib:$LD_LIBRARY_PATH
      export CFLAGS=-I$CUDA_PATH/include
      export LDFLAGS="-L$CUDA_PATH/lib64 -L$CUDA_PATH/lib"
    fi

- Install Cupy::

    sudo bash
    pip install -vvv cupy --no-cache-dir


Try Samples
-----------

You can try to run samples to check if the installation succeeded::

    roslaunch jsk_perception sample_fcn_object_segmentation.launch gpu:=0
    roslaunch jsk_perception sample_people_pose_estimation_2d.launch GPU:=0
    roslaunch jsk_perception sample_regional_feature_based_object_recognition.launch GPU:=0

Trouble Shooting
----------------

- After installing CUDA and rebooting, ``nvidia-smi`` returns ``command not found``

If your PC uses dual boot, please check BIOS setting and secure boot is disabled.

- When installing jsk_perception, ``rosdep install --from-paths --ignore-src -y -r src`` fails due to pip version:

Please make sure you have pip >= 9.0.1. If not, please try ``sudo python -m pip install pip==9.0.1``, for example. Please do not execute ``pip install -U pip``. (2018.11.20)
