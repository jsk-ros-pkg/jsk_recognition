cmake_minimum_required(VERSION 3.0.2)
project(gpt4v_vqa)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv dynamic_reconfigure)

catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3.8
    USE_SYSTEM_PACKAGES FALSE
    CHECK_VENV FALSE
)

generate_dynamic_reconfigure_options(
  cfg/GPT4V.cfg
)

catkin_package()

catkin_install_python(
  PROGRAMS
    node_scripts/gpt4v_vqa_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})