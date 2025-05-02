cmake_minimum_required(VERSION 3.5)
project(image_restoration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  catkin_virtualenv
  )

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
