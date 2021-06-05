Tips & FAQs
===========

Cannot compile jsk\_recognition because of "memory allocation error"
--------------------------------------------------------------------

``jsk_pcl_ros`` and ``jsk_perception`` requires much memory to be
compiled because of PCL. On average, each cpp file requires 2.5GB memory
to compile.

If your machine does not have enough memory, please use smaller number
of CPUs to compile

::

    catkin build -p1 -j1

How to install PCL from source?
-------------------------------

Refer to script used on Travis:

.. include:: ../.travis_before_script_pcl1.8.bash
   :literal:


How to install OpenCV from source?
----------------------------------

Refer to script used on Travis:

.. include:: ../.travis_before_script_opencv3.bash
   :literal:

How to install OpenNI2?
-----------------------

**By Apt**

::

    sudo apt-get install libopenni2-dev

**From Source**

.. code:: bash

    sudo apt-get install g++
    sudo apt-get install python
    sudo apt-get install libusb-1.0-0-dev
    sudo apt-get install libudev-dev
    sudo apt-get install openjdk-6-jdk
    sudo apt-get install freeglut3-dev
    sudo apt-get install graphviz

    git clone https://github.com/occipital/openni2
    cd openni2
    make
    export PATH=$(pwd)/Bin/x64-Release:$PATH
    export LD_LIBRARY_PATH=$(pwd)/Bin/x64-Release:$LD_LIBRARY_PATH

    # need to build openni2_camera from source
    cd ~/catkin_ws/src
    git clone https://github.com/ros-drivers/openni2_camera.git
    catkin build -iv

How to generate doc on local machine
------------------------------------

.. code:: bash
	  
   roscd jsk_recognition
   cd ../doc
   source setup.sh
   make html

Then you can see doc with ``_build/html/index.html``
