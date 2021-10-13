cmake_minimum_required(VERSION 2.8.3)
project(sound_classification)

find_package(catkin REQUIRED
  message_generation
  std_msgs
  )

add_message_files(
  FILES
  InSound.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )

install(DIRECTORY launch scripts sample_rosbag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

