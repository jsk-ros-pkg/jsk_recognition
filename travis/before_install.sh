  - echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
  - sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
  - wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  - sudo apt-get update -qq
  - sudo apt-get install -qq -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-ros
  # MongoDB hack - I don't fully understand this but its for moveit_warehouse
  - sudo apt-get remove -y mongodb mongodb-10gen
  - sudo apt-get install -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" # default actions
  ##### quick hack for missing python-tk on hrpsys/waitInput.py
  - sudo apt-get install -qq -y python-tk
  # Setup rosdep
  - sudo rosdep init
  - rosdep update
