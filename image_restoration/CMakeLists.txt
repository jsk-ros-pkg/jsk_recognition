cmake_minimum_required(VERSION 3.5)
project(image_restoration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  catkin_virtualenv
  )

find_package(Wget REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/weights)
set(MODEL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/weights/RealESRGAN_x4plus.pth)

if(NOT EXISTS ${MODEL_FILE})
  message(STATUS "Downloading RealESRGAN_x4plus.pth...")
  execute_process(
    COMMAND wget https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/RealESRGAN_x4plus.pth
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/weights
    RESULT_VARIABLE WGET_RESULT
  )
  if(NOT "${WGET_RESULT}" STREQUAL "0")
    message(FATAL_ERROR "Failed to download RealESRGAN_x4plus.pth")
  endif()
endif()

catkin_generate_virtualenv(
  PYTHON_INTERPRETER 3
  INPUT_REQUIREMENTS requirements.txt
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/image_restoration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
