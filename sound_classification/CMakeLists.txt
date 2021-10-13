cmake_minimum_required(VERSION 2.8.3)
project(sound_classification)

find_package(catkin REQUIRED
  catkin_virtualenv
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

catkin_generate_virtualenv()

file(GLOB SCRIPTS_FILES scripts/*.py)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch scripts sample_rosbag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
