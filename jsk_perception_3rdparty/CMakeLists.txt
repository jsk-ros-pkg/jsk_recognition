cmake_minimum_required(VERSION 2.8.3)
project(jsk_perception_3rdparty)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  jsk_data
  jsk_recognition_msgs
  jsk_recognition_utils
  jsk_topic_tools
  message_generation
  sensor_msgs
)

# ------------------------------------------------------------------------------------
# Catkin setup
# ------------------------------------------------------------------------------------

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/RemoveBackground.cfg
)

catkin_package(
  CATKIN_DEPENDS
  catkin_virtualenv
  jsk_recognition_msgs
  jsk_recognition_utils
  message_runtime
  sensor_msgs
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
)

# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

# ------------------------------------------------------------------------------------
# Catkin setup
# ------------------------------------------------------------------------------------

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

# ------------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------------
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/rembg_libs/rembg/README.md)
  message(WARNING "node_scripts/rembg_libs/rembg is not exists, download this")
  execute_process(COMMAND git submodule init rembg WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/rembg_libs)
  execute_process(COMMAND git submodule update rembg WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/rembg_libs)
endif()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# ------------------------------------------------------------------------------------
# Test
# ------------------------------------------------------------------------------------
message(AUTHOR_WARNING "Node/Nodelets without rostest: ${${PROJECT_NAME}_nodes_without_test}")

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-whitespace/parens,-whitespace/braces")
  roslint_cpp()
  roslint_python()
  # declare rostest
  macro(jsk_add_rostest rostest_file)
    add_rostest(${rostest_file})
    roslaunch_add_file_check(${rostest_file})
  endmacro()
  jsk_add_rostest(test/remove_background.test)
endif()
