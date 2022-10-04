cmake_minimum_required(VERSION 3.0.2)
project(jsk_vil_ros)

find_package(catkin REQUIRED
  jsk_vil_msgs
  std_msgs
  sensor_msgs
  rospy
  )

catkin_package()

file(GLOB PYTHON_SCRIPTS node_scripts/*.py)

catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
