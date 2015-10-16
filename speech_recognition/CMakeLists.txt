cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition)

find_package(catkin REQUIRED COMPONENTS
  audio_capture
  audio_common_msgs
  rospeex_core
  rospeex_if
  rospy
  speech_recognition_msgs
)

catkin_package(
)

install(DIRECTORY scripts launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
