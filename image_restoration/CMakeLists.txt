cmake_minimum_required(VERSION 2.8.3)
project(image_restoration)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo" OR 
   "$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR 
   "$ENV{ROS_DISTRO}" STREQUAL "melodic")
  message(WARNING "Skipping package for unsupported ROS distro: $ENV{ROS_DISTRO}")
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  catkin_virtualenv
  jsk_data
)

# Download weight
find_package(Wget REQUIRED)
add_custom_command(
  OUTPUT create_weights_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/weights"
)
add_custom_target(RealESRGAN_x4plus.pth ALL 
  DEPENDS create_weights_dir
  COMMAND ${WGET_EXECUTABLE} -q "https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/RealESRGAN_x4plus.pth" -O "${PROJECT_SOURCE_DIR}/weights/RealESRGAN_x4plus.pth"
)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  INPUT_REQUIREMENTS requirements.txt
)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/image_restoration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Download rosbag
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
# Install
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# Test
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
  add_rostest(test/image_restoration.test)
endif()
