cmake_minimum_required(VERSION 2.8.3)
project(audio_to_spectrogram)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/AudioAmplitudePlot.cfg
)

catkin_package()

install(DIRECTORY launch sample test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
if(NOT $ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
else()
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN "*"
    PATTERN "*/*.py" EXCLUDE
    )

  file(GLOB SCRIPT_PROGRAMS scripts/*.py)
  catkin_install_python(
    PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    )
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  add_rostest(test/audio_to_spectrogram.test)
  roslaunch_add_file_check(launch/audio_to_spectrogram.launch)
endif()
