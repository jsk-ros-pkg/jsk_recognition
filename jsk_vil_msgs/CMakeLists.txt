cmake_minimum_required(VERSION 3.0.2)
project(jsk_vil_msgs)

find_package(
  catkin REQUIRED
  message_generation
  actionlib_msgs
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  )

add_message_files(
  FILES
  ClipResult.msg
  QueryAndProbability.msg
  QuestionAndAnswerText.msg
  VQAResult.msg
  )

add_action_files(
  FILES
  VQATask.action
  ClipTask.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/OFA.cfg
  cfg/Clip.cfg
  )

catkin_package()
