cmake_minimum_required(VERSION 2.8.3)
project(sound_classification)

find_package(catkin REQUIRED
  catkin_virtualenv
  message_generation
  std_msgs
  roslaunch
  )
if (${catkin_virtualenv_VERSION} VERSION_LESS "0.3.0")
  message(STATUS "sound_classification requires catkin_virtualenv >= 0.3.0")
  return()
endif()

catkin_python_setup()

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

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python2
  INPUT_REQUIREMENTS requirements.in
  )

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

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

  file(GLOB LAUNCH_FILES launch/*.launch)
  list(REMOVE_ITEM LAUNCH_FILES "${PROJECT_SOURCE_DIR}/launch/record_audio_rosbag.launch")
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/audio_to_spectrogram.test)
  # kinetic could not run roslaunch.parent.ROSLaunchParent() see https://github.com/ros/ros_comm/issues/1734
  if (${roslaunch_VERSION} VERSION_GREATER "1.12.17")
    add_rostest(test/classify_sound.test)
  endif()
endif()
