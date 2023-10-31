Install Chainer/PyTorch with GPU Support
========================================

This documentation describes how to install Chainer/PyTorch with GPU suppport.


Requirements
------------

- Nvidia GPU (ex. K80, TitanX, GTX 1080Ti).
- Ubuntu (ex. 14.04, 16.04, 18.04).

  You can check whether your PC has a GPU by ``lspci | grep -i nvidia``.

Version Compatibilities for 18.04
---------------------------------

(Recommended) Use CUDA 9.1 from Official ubuntu repository [https://launchpad.net/ubuntu/bionic/+package/nvidia-cuda-dev](https://launchpad.net/ubuntu/bionic/+package/nvidia-cuda-dev)

- Chainer

  - chainer == 6.7.0 (last version supoprting python2. See [https://github.com/chainer/chainer/releases/tag/v6.7.0](https://github.com/chainer/chainer/releases/tag/v6.7.0)
  - cupy-cuda91 == 6.7.0 (chainer v6.7.0 requires cupy/cudnn for hardware acceleration support [https://docs.chainer.org/en/v6.7.0/install.html](https://docs.chainer.org/en/v6.7.0/install.html)

- PyTorch

  - pytorch == 1.1.0 (Latest pytorch version supporting CUDA 9.1 [https://download.pytorch.org/whl/cu90/torch_stable.html](https://download.pytorch.org/whl/cu90/torch_stable.html)
  - CUDA >= 9.0 (Minimum required version for PyTorch 1.1.0 [https://pytorch.org/get-started/previous-versions/#v110](https://pytorch.org/get-started/previous-versions/#v110)

(Experimental) Use CUDA 10.2 from Nvidia Developer's site [https://developer.nvidia.com/cuda-10.2-download-archive](https://developer.nvidia.com/cuda-10.2-download-archive)

- Chainer

  - chainer == 6.7.0 (last version supoprting python2. See [https://github.com/chainer/chainer/releases/tag/v6.7.0](https://github.com/chainer/chainer/releases/tag/v6.7.0)
  - cupy >=6.7.0,<7.0.0 (chainer v6.7.0 requires cupy/cudnn for hardware acceleration support [https://docs.chainer.org/en/v6.7.0/install.html](https://docs.chainer.org/en/v6.7.0/install.html)
  - cuDNN < 8 (cupy 6.7.0 requires cuDNN v5000= and <=v7999)
  - CUDA 10.2 (cuDNN v7.6.5 requires CUDA 10.2 [https://developer.nvidia.com/rdp/cudnn-archive](https://developer.nvidia.com/rdp/cudnn-archive))

- PyTorch

  - pytorch >= 1.4.0
  - CUDA >= 9.2 (Minimum required version for PyTorch [https://pytorch.org/get-started/previous-versions/#v140](https://pytorch.org/get-started/previous-versions/#v140)
  - Driver Version >= 396.26 (From CUDA Toolkit and Corresponding Driver Versions in [https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html)

Install CUDA
------------

- Ubuntu 14.04 : Download deb file from [https://developer.nvidia.com/cuda-downloads?target_os=Linux](https://developer.nvidia.com/cuda-downloads?target_os=Linux):

```bash
    # If you'd like to use CUDA8.0 on Ubuntu 14.04.
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb
    mv cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64-deb cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo dpkg -i cuda-repo-ubuntu1404-8-0-local-ga2_8.0.61-1_amd64.deb
    sudo apt-get update
    sudo apt-get install cuda
```

  - Add below to your `~/.bashrc`:

    ```bash
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
    ```


- Ubuntu 16.04 : Download deb file from [https://developer.nvidia.com/cuda-downloads?target_os=Linux](https://developer.nvidia.com/cuda-downloads?target_os=Linux):

```bash
    # If you'd like to use CUDA9.2 on Ubuntu 16.04.
    # Choose the green buttons on the web page like x86_64 -> Ubuntu -> version -> deb (network).
    # Excute 1-3 and then, change step 4 as follows:
    sudo apt install cuda-9-2
```

- Ubuntu 18.04 : You can use CUDA 9.1 by deafult

  ```bash
  sudo apt install nvidia-cuda-toolkit
  sudo apt install nvidia-cuda-dev

- (Experimental) Ubuntu 18.04 : CUDA 10.2 is the latest version which supports `jsk_perception`. Download deb file from https://developer.nvidia.com/cuda-downloads?target_os=Linux:

    ```bash
    # If you'd like to use CUDA10.2 on Ubuntu 18.04.
    # goto https://developer.nvidia.com/cuda-10.2-download-archive
    # Choose the green buttons on the web page like x86_64 -> Ubuntu -> version -> deb (network).
    # Excute all steps, but change the last step as follows:
    sudo apt install cuda-10-2
    ```

  - If you install CUDA from nvidia, Make sure to uninstall CUDA tools from packages.ubuntu.com

    ```bash
    sudo apt remove nvidia-cuda-toolkit
    sudo apt remove nvidia-cuda-dev
    ```

  - Also set environment variables to ~/.bashrc

    ```bash
    # set PATH for cuda 10.0 installation
    if [ -d "/usr/local/cuda-10.2/bin/" ]; then
        export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
        export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
        export CFLAGS=-I/usr/local/cuda-10.2/include
    fi
    ```

- After rebooting, you can see the memory usage of your GPU by ``nvidia-smi``

Install CUDNN
-------------

- If you install `pip install cupy-cuda91`, you do not need to install CUDNN manually. (c.f. [https://github.com/jsk-ros-pkg/jsk_visualization/issues/809](https://github.com/jsk-ros-pkg/jsk_visualization/issues/809)). Thus, default 18.04 user can use CUDA 9.1 and `cupy-cuda91==6.7.0` for `chainer==6.7.0` and you can SKIP this section.

  Installing CUDNN manually only requires for experimental user who install CUDA 10.2 manually.

- You need to login at [https://developer.nvidia.com/cudnn](https://developer.nvidia.com/cudnn)
- Go to cuDNN Download and choose version
- Download deb files of cuDNN Runtime Library and cuDNN Developer Library

```bash
   # If you'd like to install cuDNN for CUDA9.2 on Ubuntu 16.04
   # Download cuDNN v7.3.1 Runtime Library for Ubuntu16.04 (Deb)
   sudo dpkg -i libcudnn7_7.3.1.20-1+cuda9.2_amd64.deb
   # Download cuDNN v7.3.1 Developer Library for Ubuntu16.04 (Deb)
   sudo dpkg -i libcudnn7-dev_7.3.1.20-1+cuda9.2_amd64.deb
   # Download cuDNN v7.6.5 Developer Library for Ubuntu18.04 (Deb)
   sudo dpkg -i libcudnn7_7.6.5.32-1+cuda10.2_amd64.deb
   sudo dpkg -i libcudnn7-dev_7.6.5.32-1+cuda10.2_amd64.deb
```

Install Chainer
---------------

```bash
    sudo pip install chainer==6.7.0
```

Install Cupy
------------

- (Default) Chainer 6.7.0 requires CuPy 6.7.0 and if you have CUDA 9.1, you can use CuPy pre-compiled binary package.


  - Pre-compiled Install Cupy for CUDA 9.1 :

    ```bash
    sudo pip install cupy-cuda91==6.7.0
    ```

- (Experimental) If you have newer CUDA version. You need to install CuPy with source distribution. This requires CUDNN before you run `pip install cupy` .

  - Source Install Cupy for CUDA 10.2 :

    ```bash
    sudo pip install -vvv cupy --no-cache-dir
    ```


Install PyTorch
---------------

- 18.04 provides CUDA 9.1 by defualt. To install PyTorch compatible with this version, download following wheel from [https://download.pytorch.org/whl/cu90/torch_stable.html](https://download.pytorch.org/whl/cu90/torch_stable.html), and install manually.

```bash
   sudo pip install torch-1.1.0-cp27-cp27mu-linux_x86_64.whl
   sudo pip install torchvision-0.3.0-cp27-cp27mu-manylinux1_x86_64.whl
```

- (Experimental) If you manually install CUDA 10.2 manually, you can use latest PyTorch.

```bash
   sudo pip install torch==1.4.0
```

- See [https://github.com/jsk-ros-pkg/jsk_recognition/pull/2601#issuecomment-876948260](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2601#issuecomment-876948260) for more info.

Try Chainer Samples
-----------

You can try to run samples to check if the installation succeeded:

    roslaunch jsk_perception sample_fcn_object_segmentation.launch gpu:=0
    roslaunch jsk_perception sample_people_pose_estimation_2d.launch GPU:=0
    roslaunch jsk_perception sample_regional_feature_based_object_recognition.launch GPU:=0

Try PyTorch Samples
-----------

You can try to run samples to check if the installation succeeded:

    roslaunch jsk_perception sample_hand_pose_estimation_2d.launch gpu:=0

Trouble Shooting
----------------

- After installing CUDA and rebooting, ``nvidia-smi`` returns ``command not found``

If your PC uses dual boot, please check BIOS setting and secure boot is disabled.

- When installing jsk_perception, ``rosdep install --from-paths --ignore-src -y -r src`` fails due to pip version:

Please make sure you have pip >= 9.0.1. If not, please try ``sudo python -m pip install pip==9.0.1``, for example. Please do not execute ``pip install -U pip``. (2018.11.20)
