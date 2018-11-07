Install Chainer with GPU Support
================================

This documentation describes how to install Chainer with GPU suppport.


Requirements
------------

- Nvidia GPU (ex. K80, TitanX, GTX 1080Ti).
- Ubuntu (ex. 14.04, 16.04).


Install CUDA
------------

- Download deb file from https://developer.nvidia.com/cuda-downloads::

    # If you'd like to use CUDA8.0 on Ubuntu 14.04.
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb
    mv cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo apt-get update
    sudo apt-get install cuda


Install CUDNN
-------------

- Download tgz file from https://developer.nvidia.com/cudnn::

    tar zxvf cudnn-XXX.tgz  # it creates cuda/
    sudo cp cuda/include/* /usr/local/cuda/include/
    sudo cp cuda/lib64/* /usr/local/cuda/lib64/


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
