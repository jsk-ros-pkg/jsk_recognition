cmake_minimum_required(VERSION 2.8.3)
project(jsk_travis)
find_package(catkin REQUIRED rostest)
catkin_package()
add_rostest(example.test)
install(FILES rosdep-install.sh travis_jenkins.py travis.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
