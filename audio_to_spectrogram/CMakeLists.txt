cmake_minimum_required(VERSION 2.8.3)
project(audio_to_spectrogram)

find_package(catkin)
catkin_package()

install(DIRECTORY launch sample scripts test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  add_rostest(test/audio_to_spectrogram.test)
  roslaunch_add_file_check(launch/audio_to_spectrogram.launch)
endif()
