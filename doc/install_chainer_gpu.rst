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

::

    sudo bash
    pip install -vvv cupy --no-cache-dir


Try Samples
-----------

You can try to run samples to check if the installation succeeded::

    roslaunch jsk_perception sample_fcn_object_segmentation.launch gpu:=0
    roslaunch jsk_perception sample_people_pose_estimation_2d.launch GPU:=0
    roslaunch jsk_perception sample_regional_feature_based_object_recognition.launch GPU:=0
